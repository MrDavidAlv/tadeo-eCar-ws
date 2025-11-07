class TadeoCarController {
    constructor() {
        this.ws = null;
        this.currentMode = 'omnidirectional';
        this.speed = 100;
        this.isConnected = false;

        this.initWebSocket();
        this.initUI();
        this.initJoystick();
    }

    initWebSocket() {
        const connect = () => {
            this.ws = new WebSocket(`ws://${window.location.hostname}:8765`);

            this.ws.onopen = () => {
                this.isConnected = true;
                this.updateStatus(true);
                console.log('Conectado al servidor WebSocket');
            };

            this.ws.onclose = () => {
                this.isConnected = false;
                this.updateStatus(false);
                console.log('Desconectado. Reintentando en 3 segundos...');
                setTimeout(connect, 3000);
            };

            this.ws.onerror = (error) => {
                console.error('Error WebSocket:', error);
            };
        };

        connect();
    }

    updateStatus(connected) {
        const statusEl = document.getElementById('status');
        if (connected) {
            statusEl.className = 'status connected';
            statusEl.textContent = '✓ CONECTADO';
        } else {
            statusEl.className = 'status disconnected';
            statusEl.textContent = '✗ DESCONECTADO';
        }
    }

    sendCommand(command) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            command.speed = this.speed;
            this.ws.send(JSON.stringify(command));
        }
    }

    initUI() {
        // Mode buttons
        document.querySelectorAll('.mode-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
                btn.classList.add('active');

                document.querySelectorAll('.control-panel').forEach(panel => panel.classList.remove('active'));

                this.currentMode = btn.dataset.mode;
                const panelId = `${this.currentMode === 'omnidirectional' ? 'omni' : this.currentMode}-panel`;
                document.getElementById(panelId).classList.add('active');
            });
        });

        // Speed slider
        const speedSlider = document.getElementById('speedSlider');
        const speedValue = document.getElementById('speedValue');
        speedSlider.addEventListener('input', (e) => {
            this.speed = parseInt(e.target.value);
            speedValue.textContent = this.speed;
        });

        // Ackermann controls
        const steeringSlider = document.getElementById('steeringSlider');
        const throttleSlider = document.getElementById('throttleSlider');
        const steeringValue = document.getElementById('steeringValue');
        const throttleValue = document.getElementById('throttleValue');

        steeringSlider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value);
            steeringValue.textContent = value;
            this.sendAckermannCommand();
        });

        throttleSlider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value);
            throttleValue.textContent = value;
            this.sendAckermannCommand();
        });

        // Halo controls
        const angleSlider = document.getElementById('angleSlider');
        const haloSpeedSlider = document.getElementById('haloSpeedSlider');
        const angleValue = document.getElementById('angleValue');
        const haloSpeedValue = document.getElementById('haloSpeedValue');

        angleSlider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value);
            angleValue.textContent = value + '°';
            this.sendHaloCommand();
        });

        haloSpeedSlider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value);
            haloSpeedValue.textContent = value;
            this.sendHaloCommand();
        });

        // Spin control
        const spinSlider = document.getElementById('spinSlider');
        const spinValue = document.getElementById('spinValue');

        spinSlider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value);
            spinValue.textContent = value;
            this.sendSpinCommand();
        });

        // Stop button
        document.getElementById('stopBtn').addEventListener('click', () => {
            this.stop();
        });
    }

    initJoystick() {
        const joystick = document.getElementById('joystick');
        const handle = document.getElementById('joystickHandle');

        let isDragging = false;
        let centerX = 100;
        let centerY = 100;

        const updateJoystick = (clientX, clientY) => {
            const rect = joystick.getBoundingClientRect();
            centerX = rect.width / 2;
            centerY = rect.height / 2;

            let x = clientX - rect.left - centerX;
            let y = clientY - rect.top - centerY;

            const distance = Math.sqrt(x * x + y * y);
            const maxDistance = centerX - 30;

            if (distance > maxDistance) {
                const angle = Math.atan2(y, x);
                x = Math.cos(angle) * maxDistance;
                y = Math.sin(angle) * maxDistance;
            }

            handle.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;

            const normalizedX = x / maxDistance;
            const normalizedY = -y / maxDistance;

            this.sendOmniCommand(normalizedX, normalizedY);
        };

        const resetJoystick = () => {
            handle.style.transform = 'translate(-50%, -50%)';
            this.sendOmniCommand(0, 0);
        };

        handle.addEventListener('mousedown', () => isDragging = true);
        handle.addEventListener('touchstart', () => isDragging = true);

        document.addEventListener('mousemove', (e) => {
            if (isDragging) {
                updateJoystick(e.clientX, e.clientY);
            }
        });

        document.addEventListener('touchmove', (e) => {
            if (isDragging) {
                updateJoystick(e.touches[0].clientX, e.touches[0].clientY);
            }
        });

        document.addEventListener('mouseup', () => {
            if (isDragging) {
                isDragging = false;
                resetJoystick();
            }
        });

        document.addEventListener('touchend', () => {
            if (isDragging) {
                isDragging = false;
                resetJoystick();
            }
        });
    }

    sendOmniCommand(x, y) {
        this.sendCommand({
            mode: 'omnidirectional',
            x: x,
            y: y
        });
    }

    sendAckermannCommand() {
        const steering = parseInt(document.getElementById('steeringSlider').value) / 100.0;
        const throttle = parseInt(document.getElementById('throttleSlider').value) / 100.0;

        this.sendCommand({
            mode: 'ackermann',
            steering: steering,
            throttle: throttle
        });
    }

    sendHaloCommand() {
        const angle = parseInt(document.getElementById('angleSlider').value);
        const speed = parseInt(document.getElementById('haloSpeedSlider').value) / 100.0;

        this.sendCommand({
            mode: 'halo',
            globalAngle: angle,
            speed: speed
        });
    }

    sendSpinCommand() {
        const spinSpeed = parseInt(document.getElementById('spinSlider').value) / 100.0;

        this.sendCommand({
            mode: 'spin',
            spinSpeed: spinSpeed
        });
    }

    stop() {
        this.sendCommand({ mode: 'stop' });

        // Reset all sliders
        document.getElementById('steeringSlider').value = 0;
        document.getElementById('throttleSlider').value = 0;
        document.getElementById('haloSpeedSlider').value = 0;
        document.getElementById('spinSlider').value = 0;

        document.getElementById('steeringValue').textContent = '0';
        document.getElementById('throttleValue').textContent = '0';
        document.getElementById('haloSpeedValue').textContent = '0';
        document.getElementById('spinValue').textContent = '0';
    }
}

// Initialize controller when page loads
window.addEventListener('DOMContentLoaded', () => {
    new TadeoCarController();
});
