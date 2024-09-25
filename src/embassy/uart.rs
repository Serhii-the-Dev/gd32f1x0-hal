use core::{
    future::poll_fn,
    ops::Deref,
    slice,
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
    task::Poll,
};

use cortex_m::peripheral::NVIC;
use embassy_hal_internal::atomic_ring_buffer::RingBuffer;
use embassy_sync::waitqueue::AtomicWaker;

use crate::{
    pac::{Interrupt, Usart0, interrupt, usart0::RegisterBlock},
    rcu::{Clocks, Enable, GetBusFreq, Reset, sealed::RcuBus},
    serial::{Config, Error, RxPin, TxPin, UsartConfigExt},
};

trait Instance {
    fn register() -> &'static RegisterBlock;
    fn state() -> &'static State;
    fn interrupt() -> Interrupt;
}

impl Instance for Usart0 {
    fn register() -> &'static RegisterBlock {
        unsafe { &*Self::PTR }
    }

    fn state() -> &'static State {
        static STATE: State = State::new();
        &STATE
    }

    fn interrupt() -> Interrupt {
        Interrupt::USART0
    }
}

struct State {
    rx_waker: AtomicWaker,
    rx_buf: RingBuffer,
    tx_waker: AtomicWaker,
    tx_buf: RingBuffer,
    tx_done: AtomicBool,
    // TODO: add refcount in case of UART re-use
    tx_rx_refcount: AtomicU8,
}

impl State {
    const fn new() -> Self {
        Self {
            rx_buf: RingBuffer::new(),
            tx_buf: RingBuffer::new(),
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            tx_done: AtomicBool::new(true),
            tx_rx_refcount: AtomicU8::new(0),
        }
    }
}

/// Tx-only buffered UART
///
/// Created with [BufferedUart::split]
pub struct BufferedUartRx {
    state: &'static State,
    interrupt: Interrupt,
}

/// Rx-only buffered UART
///
/// Created with [BufferedUart::split]
pub struct BufferedUartTx<'d> {
    state: &'static State,
    interrupt: Interrupt,
    register: &'d RegisterBlock,
}

pub struct BufferedUart<'d> {
    rx: BufferedUartRx,
    tx: BufferedUartTx<'d>,
}

#[allow(private_bounds)]
impl BufferedUartRx {
    pub fn new<USART: Instance>() -> Self {
        Self {
            state: USART::state(),
            interrupt: USART::interrupt(),
        }
    }

    fn read_ready(&mut self) -> Result<bool, Error> {
        let state = self.state;

        Ok(!state.rx_buf.is_empty())
    }

    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        poll_fn(move |cx| {
            let state = self.state;
            let mut rx_reader = unsafe { state.rx_buf.reader() };
            let mut buf_len = 0;
            let mut data = rx_reader.pop_slice();

            while !data.is_empty() && buf_len < buf.len() {
                let data_len = data.len().min(buf.len() - buf_len);
                buf[buf_len..buf_len + data_len].copy_from_slice(&data[..data_len]);
                buf_len += data_len;

                let do_pend = state.rx_buf.is_full();
                rx_reader.pop_done(data_len);

                if do_pend {
                    NVIC::pend(self.interrupt);
                }

                data = rx_reader.pop_slice();
            }

            if buf_len != 0 {
                Poll::Ready(Ok(buf_len))
            } else {
                state.rx_waker.register(cx.waker());
                Poll::Pending
            }
        })
        .await
    }

    async fn fill_buf(&self) -> Result<&[u8], Error> {
        poll_fn(move |cx| {
            let state = self.state;
            let mut rx_reader = unsafe { state.rx_buf.reader() };
            let (p, n) = rx_reader.pop_buf();
            if n == 0 {
                state.rx_waker.register(cx.waker());
                return Poll::Pending;
            }

            let buf = unsafe { slice::from_raw_parts(p, n) };
            Poll::Ready(Ok(buf))
        })
        .await
    }

    fn consume(&self, amt: usize) {
        let state = self.state;
        let mut rx_reader = unsafe { state.rx_buf.reader() };
        let full = state.rx_buf.is_full();
        rx_reader.pop_done(amt);
        if full {
            NVIC::pend(self.interrupt);
        }
    }
}

#[allow(private_bounds)]
impl<'d> BufferedUartTx<'d> {
    pub fn new<USART: Instance>() -> Self {
        Self {
            state: USART::state(),
            interrupt: USART::interrupt(),
            register: USART::register(),
        }
    }

    async fn write(&self, buf: &[u8]) -> Result<usize, Error> {
        let r = self.register;

        if r.ctl2().read().hden().is_selected() && r.ctl0().read().ten().is_disabled() {
            r.ctl0().modify(|_, w| w.ten().enabled().ren().disabled());
        }

        poll_fn(move |cx| {
            let state = self.state;
            state.tx_done.store(false, Ordering::Release);

            let empty = state.tx_buf.is_empty();

            let mut tx_writer = unsafe { state.tx_buf.writer() };
            let data = tx_writer.push_slice();
            if data.is_empty() {
                state.tx_waker.register(cx.waker());
                return Poll::Pending;
            }

            let n = data.len().min(buf.len());
            data[..n].copy_from_slice(&buf[..n]);
            tx_writer.push_done(n);

            if empty {
                NVIC::pend(self.interrupt);
            }

            Poll::Ready(Ok(n))
        })
        .await
    }

    async fn flush(&self) -> Result<(), Error> {
        poll_fn(move |cx| {
            let state = self.state;

            if !state.tx_done.load(Ordering::Acquire) {
                state.tx_waker.register(cx.waker());
                return Poll::Pending;
            }

            let r = self.register;
            if r.ctl2().read().hden().is_selected() {
                r.ctl0().modify(|_, w| w.ren().enabled());
            }

            Poll::Ready(Ok(()))
        })
        .await
    }
}

// TODO: iterate through available USARTs via a macro
#[interrupt]
fn USART0() {
    on_interrupt(Usart0::register(), Usart0::state());
}

fn on_interrupt(register: &RegisterBlock, state: &'static State) {
    // info!("Interrupt");
    let status = register.stat().read();

    let data = if status.rbne().bit() || status.idlef().bit() || status.orerr().bit() {
        Some(register.rdata().read().rdata().bits() as u8)
    } else {
        None
    };

    register.intc().write(|w| unsafe { w.bits(status.bits()) });

    // TODO: Some error handling maybe?
    if status.perr().bit() {
        // warn!("Parity error");
    }
    if status.ferr().bit() {
        // warn!("Framing error");
    }
    if status.nerr().bit() {
        // warn!("Noise error");
    }
    if status.orerr().bit() {
        // warn!("Overrun error");
    }

    if status.rbne().bit() {
        let mut rx_writer = unsafe { state.rx_buf.writer() };
        let buf = rx_writer.push_slice();
        if !buf.is_empty() {
            if let Some(byte) = data {
                buf[0] = byte;
                rx_writer.push_done(1);
            }
        }

        if !state.rx_buf.is_empty() {
            state.rx_waker.wake();
        }
    }

    if status.idlef().bit() {
        state.rx_waker.wake();
    }

    if status.tc().bit() {
        register.ctl0().modify(|_, w| w.tcie().disabled());

        state.tx_done.store(true, Ordering::Release);
        state.tx_waker.wake();
    }

    if register.stat().read().tbe().bit() {
        let mut tx_reader = unsafe { state.tx_buf.reader() };
        let buf = tx_reader.pop_slice();
        if !buf.is_empty() {
            register.ctl0().modify(|_, w| w.tbeie().enabled());

            // Enable transmission complete interrupt when last byte is going to be sent out.
            if buf.len() == 1 {
                register.ctl0().modify(|_, w| w.tcie().enabled());
            }

            register
                .tdata()
                .write(|w| unsafe { w.tdata().bits(buf[0].into()) });
            tx_reader.pop_done(1);
        } else {
            // Disable interrupt until we have something to transmit again.
            register.ctl0().modify(|_, w| w.tbeie().disabled());
        }
    }
}

#[allow(private_bounds)]
impl<'d> BufferedUart<'d> {
    pub fn new<USART: Instance>(
        _instance: USART,
        _rx_pin: impl RxPin<USART>,
        _tx_pin: impl TxPin<USART>,
    ) -> Self {
        todo!();
    }

    pub fn new_half_duplex<
        USART: Instance + RcuBus + Enable + Reset + Deref<Target = RegisterBlock>,
    >(
        _instance: USART,
        _pin: impl TxPin<USART>,
        tx_buffer: &mut [u8],
        rx_buffer: &mut [u8],
        config: Config,
        clocks: Clocks,
        bus: &mut USART::Bus,
    ) -> Self
    where
        USART::Bus: GetBusFreq,
    {
        let register = USART::register();

        _instance.enable_configure(config, clocks, bus);

        let result = Self::new_inner(_instance);
        let state = result.tx.state;

        let len = tx_buffer.len();
        unsafe { state.tx_buf.init(tx_buffer.as_mut_ptr(), len) };
        let len = rx_buffer.len();
        unsafe { state.rx_buf.init(rx_buffer.as_mut_ptr(), len) };

        register.ctl2().modify(|_, w| w.hden().selected());

        // Receiver should be enabled by default in the half-duplex mode
        register
            .ctl0()
            .modify(|_, w| w.ren().enabled().ten().disabled().uen().enabled());

        result
    }

    fn new_inner<USART: Instance>(_instance: USART) -> Self {
        let state = USART::state();
        let interrupt = USART::interrupt();
        let register = USART::register();

        register
            .ctl0()
            .modify(|_, w| w.rbneie().enabled().idleie().enabled());

        NVIC::unpend(interrupt);
        unsafe {
            NVIC::unmask(interrupt);
        }

        Self {
            rx: BufferedUartRx {
                state,
                interrupt: USART::interrupt(),
            },
            tx: BufferedUartTx {
                state,
                interrupt: USART::interrupt(),
                register: USART::register(),
            },
        }
    }
}

impl<'d> embedded_io_async::ErrorType for BufferedUart<'d> {
    type Error = Error;
}

impl embedded_io_async::ErrorType for BufferedUartRx {
    type Error = Error;
}

impl<'d> embedded_io_async::ErrorType for BufferedUartTx<'d> {
    type Error = Error;
}

impl<'d> embedded_io_async::Read for BufferedUart<'d> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf).await
    }
}

impl embedded_io_async::Read for BufferedUartRx {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        Self::read(self, buf).await
    }
}

impl<'d> embedded_io_async::Write for BufferedUart<'d> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush().await
    }
}

impl<'d> embedded_io_async::Write for BufferedUartTx<'d> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Self::write(self, buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Self::flush(self).await
    }
}
