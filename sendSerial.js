const SerialPort = require('serialport');
const { EventEmitter } = require('events');

class SerialSender extends EventEmitter {
    constructor(path, baudRate = 9600) {
        super();
        this.port = new SerialPort({ path, baudRate });
        this.queue = [];
        this.isProcessing = false;
        this.DELAY_MS = 20;
        this.lastSentString = null;  // Track last sent string

        // Handle port errors
        this.port.on('error', (err) => {
            console.error('Serial port error:', err);
            this.emit('error', err);
        });

        // Handle port close
        this.port.on('close', () => {
            console.log('Serial port closed');
            this.emit('close');
        });
    }

    // Modified send method to check for duplicates
    async send(str) {
        // Return early if string matches last sent string
        if (str === this.lastSentString) {
            return;
        }

        // Update last sent string
        this.lastSentString = str;

        // Convert string to array of bytes
        const bytes = Array.from(str).map(char => char.charCodeAt(0));
        return this.sendBytes(bytes);
    }

    async sendBytes(bytes) {
        return new Promise((resolve, reject) => {
            // Add the bytes to the queue with their resolve/reject callbacks
            this.queue.push({
                bytes,
                resolve,
                reject
            });

            // Start processing if not already processing
            if (!this.isProcessing) {
                this.processQueue();
            }
        });
    }

    async processQueue() {
        if (this.isProcessing || this.queue.length === 0) {
            return;
        }

        this.isProcessing = true;

        try {
            while (this.queue.length > 0) {
                const { bytes, resolve, reject } = this.queue[0];

                try {
                    // Send each byte with a delay
                    for (const byte of bytes) {
                        await this.delay(this.DELAY_MS);
                        await this.writeByte(byte);
                    }

                    // Remove the processed item from the queue
                    this.queue.shift();
                    resolve();
                } catch (error) {
                    // Remove the failed item from the queue
                    this.queue.shift();
                    reject(error);
                }
            }
        } finally {
            this.isProcessing = false;
        }
    }

    writeByte(byte) {
        return new Promise((resolve, reject) => {
            this.port.write(Buffer.from([byte]), (err) => {
                if (err) {
                    reject(err);
                } else {
                    resolve();
                }
            });
        });
    }

    delay(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }

    close() {
        return new Promise((resolve, reject) => {
            this.port.close((err) => {
                if (err) {
                    reject(err);
                } else {
                    resolve();
                }
            });
        });
    }
}

// Example usage:

const sender = new SerialSender('COM3');  // or whatever your port is

const motorXYZ = {x: 1, y: 2, z: 18}
sender.sendBytes([
    '%'.charCodeAt(0),
    'j'.charCodeAt(0),
    motorXYZ.x,
    motorXYZ.y,
    motorXYZ.z,
    '!'.charCodeAt(0)
])


// Process 1
sender.sendBytes([0x01, 0x02, 0x03])
    .then(() => console.log('Process 1 bytes sent'))
    .catch(err => console.error('Process 1 error:', err));

// Process 2 (will wait for Process 1 to finish)
sender.sendBytes([0x04, 0x05, 0x06])
    .then(() => console.log('Process 2 bytes sent'))
    .catch(err => console.error('Process 2 error:', err));


module.exports = SerialSender;
