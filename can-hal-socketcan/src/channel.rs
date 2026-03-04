use std::io;
use std::time::{Duration, Instant};

use socketcan::frame::CanAnyFrame;
use socketcan::{CanFdSocket, Socket, SocketOptions};

use can_hal::channel::{Receive, ReceiveFd, Transmit, TransmitFd};
use can_hal::filter::{Filter, Filterable};
use can_hal::frame::{CanFdFrame, CanFrame, Frame, Timestamped};

use crate::convert;
use crate::error::SocketCanError;

/// A CAN channel backed by a Linux SocketCAN FD socket.
///
/// Implements `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, and `Filterable`.
///
/// Created via [`SocketCanDriver`](crate::SocketCanDriver) or
/// [`SocketCanChannel::open`].
pub struct SocketCanChannel {
    socket: CanFdSocket,
    nonblocking: bool,
}

impl SocketCanChannel {
    /// Open a channel on the named interface (e.g., `"vcan0"`, `"can0"`).
    pub fn open(ifname: &str) -> Result<Self, SocketCanError> {
        let socket = CanFdSocket::open(ifname)?;
        Ok(Self {
            socket,
            nonblocking: false,
        })
    }

    /// Open a channel by kernel interface index.
    pub fn open_iface(ifindex: u32) -> Result<Self, SocketCanError> {
        let socket = CanFdSocket::open_iface(ifindex)?;
        Ok(Self {
            socket,
            nonblocking: false,
        })
    }

    fn ensure_blocking(&mut self) -> Result<(), SocketCanError> {
        if self.nonblocking {
            self.socket.set_nonblocking(false)?;
            self.nonblocking = false;
        }
        Ok(())
    }

    fn ensure_nonblocking(&mut self) -> Result<(), SocketCanError> {
        if !self.nonblocking {
            self.socket.set_nonblocking(true)?;
            self.nonblocking = true;
        }
        Ok(())
    }
}

impl Transmit for SocketCanChannel {
    type Error = SocketCanError;

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
        let sc_frame = convert::to_socketcan_data_frame(frame)?;
        self.socket.write_frame(&sc_frame)?;
        Ok(())
    }
}

impl Receive for SocketCanChannel {
    type Error = SocketCanError;

    fn receive(&mut self) -> Result<Timestamped<CanFrame>, Self::Error> {
        self.ensure_blocking()?;
        loop {
            let any_frame = self.socket.read_frame()?;
            let now = Instant::now();
            if let CanAnyFrame::Normal(data_frame) = any_frame {
                let frame = convert::from_socketcan_data_frame(&data_frame)?;
                return Ok(Timestamped::new(frame, now));
            }
            // Skip FD, remote, and error frames — caller wants classic only.
        }
    }

    fn try_receive(&mut self) -> Result<Option<Timestamped<CanFrame>>, Self::Error> {
        self.ensure_nonblocking()?;
        match self.socket.read_frame() {
            Ok(CanAnyFrame::Normal(data_frame)) => {
                let now = Instant::now();
                let frame = convert::from_socketcan_data_frame(&data_frame)?;
                Ok(Some(Timestamped::new(frame, now)))
            }
            Ok(_) => Ok(None),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(SocketCanError::Io(e)),
        }
    }

    fn receive_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<CanFrame>>, Self::Error> {
        self.ensure_blocking()?;
        self.socket.set_read_timeout(timeout)?;
        let deadline = Instant::now() + timeout;
        let result = loop {
            match self.socket.read_frame() {
                Ok(CanAnyFrame::Normal(data_frame)) => {
                    let now = Instant::now();
                    break convert::from_socketcan_data_frame(&data_frame)
                        .map(|f| Some(Timestamped::new(f, now)));
                }
                Ok(_) => {
                    // Skip non-classic frames. Update remaining timeout.
                    let now = Instant::now();
                    if now >= deadline {
                        break Ok(None);
                    }
                    let _ = self.socket.set_read_timeout(deadline - now);
                    continue;
                }
                Err(e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    break Ok(None);
                }
                Err(e) => break Err(SocketCanError::Io(e)),
            }
        };
        // Restore to no timeout (infinite blocking).
        let _ = self.socket.set_read_timeout(None);
        result
    }
}

impl TransmitFd for SocketCanChannel {
    type Error = SocketCanError;

    fn transmit_fd(&mut self, frame: &CanFdFrame) -> Result<(), Self::Error> {
        let sc_frame = convert::to_socketcan_fd_frame(frame)?;
        self.socket.write_frame(&sc_frame)?;
        Ok(())
    }
}

impl ReceiveFd for SocketCanChannel {
    type Error = SocketCanError;

    fn receive_fd(&mut self) -> Result<Timestamped<Frame>, Self::Error> {
        self.ensure_blocking()?;
        loop {
            let any_frame = self.socket.read_frame()?;
            let now = Instant::now();
            match convert::from_socketcan_any_frame(any_frame) {
                Ok(frame) => return Ok(Timestamped::new(frame, now)),
                Err(_) => continue, // Skip remote/error frames.
            }
        }
    }

    fn try_receive_fd(&mut self) -> Result<Option<Timestamped<Frame>>, Self::Error> {
        self.ensure_nonblocking()?;
        match self.socket.read_frame() {
            Ok(any_frame) => {
                let now = Instant::now();
                match convert::from_socketcan_any_frame(any_frame) {
                    Ok(frame) => Ok(Some(Timestamped::new(frame, now))),
                    Err(_) => Ok(None),
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(SocketCanError::Io(e)),
        }
    }

    fn receive_fd_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<Frame>>, Self::Error> {
        self.ensure_blocking()?;
        self.socket.set_read_timeout(timeout)?;
        let deadline = Instant::now() + timeout;
        let result = loop {
            match self.socket.read_frame() {
                Ok(any_frame) => {
                    let now = Instant::now();
                    match convert::from_socketcan_any_frame(any_frame) {
                        Ok(frame) => break Ok(Some(Timestamped::new(frame, now))),
                        Err(_) => {
                            // Skip remote/error frames. Update remaining timeout.
                            if now >= deadline {
                                break Ok(None);
                            }
                            let _ = self.socket.set_read_timeout(deadline - now);
                            continue;
                        }
                    }
                }
                Err(e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    break Ok(None);
                }
                Err(e) => break Err(SocketCanError::Io(e)),
            }
        };
        let _ = self.socket.set_read_timeout(None);
        result
    }
}

impl Filterable for SocketCanChannel {
    type Error = SocketCanError;

    fn set_filters(&mut self, filters: &[Filter]) -> Result<(), Self::Error> {
        let sc_filters: Vec<_> = filters.iter().map(convert::to_socketcan_filter).collect();
        self.socket.set_filters(&sc_filters)?;
        Ok(())
    }

    fn clear_filters(&mut self) -> Result<(), Self::Error> {
        self.socket.set_filter_accept_all()?;
        Ok(())
    }
}
