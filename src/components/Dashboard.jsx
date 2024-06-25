/*
import React, { Component } from 'react';
import ROSLIB from 'roslib';
import Alert from 'react-bootstrap/Alert';
import Spinner from 'react-bootstrap/Spinner';

class Dashboard extends Component {
    constructor(props) {
        super(props);
        this.state = {
            temperatures: Array(5).fill(null),
            pressures: Array(5).fill(null),
            gasDetected: null,
            motorStatus: 'unknown',
            alarmStatus: 'inactive',
            cameraImage: null,
            rosConnected: Array(10).fill(false), // 5 for temperature, 5 for pressure
            gasConnected: false,
            motorConnected: false,
            alarmConnected: false,
        };
        this.ros = new ROSLIB.Ros();
    }

    componentDidMount() {
        this.connectRos();
    }

    connectRos() {
        this.ros.on('connection', () => {
            console.log('Connected to ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => true) });
            this.setupListeners();
            this.setupCameraListener();
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to ROS:', error);
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        this.ros.on('close', () => {
            console.log('Disconnected from ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        // Adjust according to your ROSBridge server address
        this.ros.connect('ws://192.168.0.119:9090');
    }

    setupListeners() {
        for (let i = 0; i < 5; i++) {
            const temperatureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/temperature_${i}`,
                messageType: 'std_msgs/Float64'
            });

            temperatureListener.subscribe((message) => {
                console.log(`Temperature ${i}:`, message.data);
                const newTemperatures = [...this.state.temperatures];
                newTemperatures[i] = message.data;
                this.setState({ temperatures: newTemperatures });
            });

            const pressureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/pressure_${i}`,
                messageType: 'std_msgs/Float64'
            });

            pressureListener.subscribe((message) => {
                console.log(`Pressure ${i}:`, message.data);
                const newPressures = [...this.state.pressures];
                newPressures[i] = message.data;
                this.setState({ pressures: newPressures });
            });
        }

        const gasDetectedListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/gas_detected',
            messageType: 'std_msgs/Bool'
        });

        gasDetectedListener.subscribe((message) => {
            console.log('Gas Detected:', message.data);
            this.setState({ gasDetected: message.data, gasConnected: true });
        });

        const motorStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor_status',
            messageType: 'std_msgs/String'
        });

        motorStatusListener.subscribe((message) => {
            console.log('Motor Status:', message.data);
            this.setState({ motorStatus: message.data, motorConnected: true });
        });

        const alarmStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/alarm_status',
            messageType: 'std_msgs/String'
        });

        alarmStatusListener.subscribe((message) => {
            console.log('Alarm Status:', message.data);
            this.setState({ alarmStatus: message.data, alarmConnected: true });
        });
    }

    setupCameraListener() {
        const imageListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/camera/image_raw',
            messageType: 'sensor_msgs/Image'
        });

        imageListener.subscribe((message) => {
            console.log('Camera Image Received');
            const imageUrl = `data:image/jpeg;base64,${this.arrayBufferToBase64(message.data.data)}`; // Access .data property
            this.setState({ cameraImage: imageUrl });
        });
    }

    arrayBufferToBase64(buffer) {
        let binary = '';
        const bytes = new Uint8Array(buffer);
        const len = bytes.byteLength;

        for (let i = 0; i < len; i++) {
            binary += String.fromCharCode(bytes[i]);
        }

        return window.btoa(binary);
    }

    render() {
        const { temperatures, pressures, gasDetected, motorStatus, alarmStatus, cameraImage, rosConnected, gasConnected, motorConnected, alarmConnected } = this.state;

        return (
            <div>
                <h1>Robot Dashboard</h1>

                <Alert variant={rosConnected.includes(true) ? 'success' : 'danger'}>
                    {rosConnected.includes(true) ? 'Connected to ROS' : 'Disconnected from ROS'}
                </Alert>

                <h2>Temperature Sensors</h2>
                {temperatures.map((temp, index) => (
                    <Alert key={`temp-${index}`} variant={temp > 80 ? 'danger' : (temp < 20 ? 'info' : 'success')}>
                        {`Temperature Sensor ${index + 1}: `}
                        {temp !== null ? `${temp} °C` : <Spinner animation="border" size="sm" />}
                        <br />
                        {rosConnected[index] ? 'Connected ' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Pressure Sensors</h2>
                {pressures.map((pressure, index) => (
                    <Alert key={`pressure-${index}`} variant={pressure !== null ? 'success' : 'danger'}>
                        {`Pressure Sensor ${index + 1}: `}
                        {pressure !== null ? `${pressure} units` : <Spinner animation="border" size="sm" />}
                        <br />
                        {rosConnected[index + 5] ? 'Connected' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Gas</h2>
                <Alert variant={gasDetected ? 'danger' : 'success'}>
                    Gas Detection: {gasDetected !== null ? (gasDetected ? 'Detected' : 'Not Detected') : <Spinner animation="border" size="sm" />}
                    <br />
                    {gasConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                <h2>Motor</h2>
                <Alert variant={motorStatus === 'running' ? 'success' : 'danger'}>
                    Motor Status: {motorStatus !== 'unknown' ? motorStatus : <Spinner animation="border" size="sm" />}
                    <br />
                    {motorConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                <h2>Alarm</h2>
                <Alert variant={alarmStatus === 'active' ? 'danger' : 'success'}>
                    Alarm Status: {alarmStatus !== 'inactive' ? alarmStatus : <Spinner animation="border" size="sm" />}
                    <br />
                    {alarmConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                {cameraImage ? (
                    <div>
                        <h2>Live Camera Feed</h2>
                        <img src={cameraImage} alt="Camera Feed" style={{ maxWidth: '100%' }} />
                    </div>
                ) : (
                    <Spinner animation="border" />
                )}
            </div>
        );
    }
}

export default Dashboard;
*/

/*
import React, { Component } from 'react';
import ROSLIB from 'roslib';
import Alert from 'react-bootstrap/Alert';
import Button from 'react-bootstrap/Button';
import Spinner from 'react-bootstrap/Spinner';

class Dashboard extends Component {
    constructor(props) {
        super(props);
        this.state = {
            temperatures: Array(5).fill(null),
            pressures: Array(5).fill(null),
            gasDetected: null,
            motorStatus: 'unknown',
            alarmStatus: 'inactive',
            cameraImage: null,
            rosConnected: Array(10).fill(false), // 5 for temperature, 5 for pressure
            gasConnected: false,
            motorConnected: false,
            alarmConnected: false,
        };
        this.ros = new ROSLIB.Ros();
    }

    componentDidMount() {
        this.connectRos();
    }

    connectRos() {
        this.ros.on('connection', () => {
            console.log('Connected to ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => true) });
            this.setupListeners();
            this.setupCameraListener();
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to ROS:', error);
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        this.ros.on('close', () => {
            console.log('Disconnected from ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        // Adjust according to your ROSBridge server address
        this.ros.connect('ws://192.168.0.119:9090');
    }

    setupListeners() {
        for (let i = 0; i < 5; i++) {
            const temperatureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/temperature_${i}`,
                messageType: 'std_msgs/Float64'
            });

            temperatureListener.subscribe((message) => {
                console.log(`Temperature ${i}:`, message.data);
                const newTemperatures = [...this.state.temperatures];
                newTemperatures[i] = message.data;
                this.setState({ temperatures: newTemperatures }, this.checkAutomation);
            });

            const pressureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/pressure_${i}`,
                messageType: 'std_msgs/Float64'
            });

            pressureListener.subscribe((message) => {
                console.log(`Pressure ${i}:`, message.data);
                const newPressures = [...this.state.pressures];
                newPressures[i] = message.data;
                this.setState({ pressures: newPressures });
            });
        }

        const gasDetectedListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/gas_detected',
            messageType: 'std_msgs/Bool'
        });

        gasDetectedListener.subscribe((message) => {
            console.log('Gas Detected:', message.data);
            this.setState({ gasDetected: message.data, gasConnected: true }, this.checkAutomation);
        });

        const motorStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor_status',
            messageType: 'std_msgs/String'
        });

        motorStatusListener.subscribe((message) => {
            console.log('Motor Status:', message.data);
            this.setState({ motorStatus: message.data, motorConnected: true });
        });

        const alarmStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/alarm_status',
            messageType: 'std_msgs/String'
        });

        alarmStatusListener.subscribe((message) => {
            console.log('Alarm Status:', message.data);
            this.setState({ alarmStatus: message.data, alarmConnected: true });
        });
    }

    setupCameraListener() {
        const imageListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/camera/image_raw',
            messageType: 'sensor_msgs/Image'
        });

        imageListener.subscribe((message) => {
            console.log('Camera Image Received');
            const imageUrl = `data:image/jpeg;base64,${this.arrayBufferToBase64(message.data.data)}`;
            this.setState({ cameraImage: imageUrl });
        });
    }

    arrayBufferToBase64(buffer) {
        let binary = '';
        const bytes = new Uint8Array(buffer);
        const len = bytes.byteLength;

        for (let i = 0; i < len; i++) {
            binary += String.fromCharCode(bytes[i]);
        }

        return window.btoa(binary);
    }

    sendMotorCommand = (command) => {
        const motorCommandPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/control/motor',  // Ensure this matches the backend topic name
            messageType: 'std_msgs/String'
        });

        const message = new ROSLIB.Message({
            data: command
        });

        motorCommandPublisher.publish(message);
    }

    checkAutomation = () => {
        const { temperatures, gasDetected } = this.state;

        if (temperatures.some(temp => temp > 80)) {
            this.sendMotorCommand('off');  // Updated command
        }

        if (gasDetected) {
            this.sendMotorCommand('off');  // Updated command
            this.sendAlarmCommand('activate');
        } else {
            this.sendAlarmCommand('deactivate');
        }
    }

    sendAlarmCommand = (command) => {
        const alarmCommandPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/alarm_command',
            messageType: 'std_msgs/String'
        });

        const message = new ROSLIB.Message({
            data: command
        });

        alarmCommandPublisher.publish(message);
    }

    render() {
        const { temperatures, pressures, gasDetected, motorStatus, alarmStatus, cameraImage, rosConnected, gasConnected, motorConnected, alarmConnected } = this.state;

        return (
            <div>
                <h1>Robot Dashboard</h1>

                <Alert variant={rosConnected.includes(true) ? 'success' : 'danger'}>
                    {rosConnected.includes(true) ? 'Connected to ROS' : 'Disconnected from ROS'}
                </Alert>

                <h2>Temperature Sensors</h2>
                {temperatures.map((temp, index) => (
                    <Alert key={`temp-${index}`} variant={temp > 80 ? 'danger' : (temp < 20 ? 'info' : 'success')}>
                        {`Temperature Sensor ${index + 1}: `}
                        {temp !== null ? `${temp} °C` : <Spinner animation="border" size="sm" />}
                        <br />
                        {rosConnected[index] ? 'Connected ' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Pressure Sensors</h2>
                {pressures.map((pressure, index) => (
                    <Alert key={`pressure-${index}`} variant={pressure !== null ? 'success' : 'danger'}>
                        {`Pressure Sensor ${index + 1}: `}
                        {pressure !== null ? `${pressure} units` : <Spinner animation="border" size="sm" />}
                        <br />
                        {rosConnected[index + 5] ? 'Connected' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Gas</h2>
                <Alert variant={gasDetected ? 'danger' : 'success'}>
                    Gas Detection: {gasDetected !== null ? (gasDetected ? 'Detected' : 'Not Detected') : <Spinner animation="border" size="sm" />}
                    <br />
                    {gasConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                <h2>Motor</h2>
                <Alert variant={motorStatus === 'running' ? 'success' : 'danger'}>
                    Motor Status: {motorStatus !== 'unknown' ? motorStatus : <Spinner animation="border" size="sm" />}
                    <br />
                    {motorConnected ? 'Connected' : 'Not Connected'}
                </Alert>
                <Button variant="primary" onClick={() => this.sendMotorCommand('on')}>Turn On Motor</Button>
                <Button variant="danger" onClick={() => this.sendMotorCommand('off')}>Turn Off Motor</Button>

                <h2>Alarm</h2>
                <Alert variant={alarmStatus === 'active' ? 'danger' : 'success'}>
                    Alarm Status: {alarmStatus !== 'inactive' ? alarmStatus : <Spinner animation="border" size="sm" />}
                    <br />
                    {alarmConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                {cameraImage ? (
                    <div>
                        <h2>Live Camera Feed</h2>
                        <img src={cameraImage} alt="Camera Feed" style={{ maxWidth: '100%' }} />
                    </div>
                ) : (
                    <Spinner animation="border" />
                )}
            </div>
        );
    }
}

export default Dashboard;
*/
/*
import React, { Component } from 'react';
import ROSLIB from 'roslib';
import Alert from 'react-bootstrap/Alert';
import Button from 'react-bootstrap/Button';
import Spinner from 'react-bootstrap/Spinner';

class Dashboard extends Component {
    constructor(props) {
        super(props);
        this.state = {
            temperatures: Array(5).fill(null),
            pressures: Array(5).fill(null),
            gasDetected: null,
            motorStatus: 'unknown',
            alarmStatus: 'inactive',
            cameraImage: null,
            rosConnected: Array(10).fill(false),
            gasConnected: false,
            motorConnected: false,
            alarmConnected: false,
            visualAlert: ''  // State to hold visual alert messages
        };
        this.ros = new ROSLIB.Ros();
    }

    componentDidMount() {
        this.connectRos();
    }

    connectRos() {
        this.ros.on('connection', () => {
            console.log('Connected to ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => true) });
            this.setupListeners();
            this.setupCameraListener();
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to ROS:', error);
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        this.ros.on('close', () => {
            console.log('Disconnected from ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        // Adjust according to your ROSBridge server address
        this.ros.connect('ws://192.168.0.119:9090');
    }

    setupListeners() {
        for (let i = 0; i < 5; i++) {
            const temperatureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/temperature_${i}`,
                messageType: 'std_msgs/Float64'
            });

            temperatureListener.subscribe((message) => {
                console.log(`Temperature ${i}:`, message.data);
                const newTemperatures = [...this.state.temperatures];
                newTemperatures[i] = message.data;
                this.setState({ temperatures: newTemperatures });
            });

            const pressureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/pressure_${i}`,
                messageType: 'std_msgs/Float64'
            });

            pressureListener.subscribe((message) => {
                console.log(`Pressure ${i}:`, message.data);
                const newPressures = [...this.state.pressures];
                newPressures[i] = message.data;
                this.setState({ pressures: newPressures });
            });
        }

        const gasDetectedListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/gas_detected',
            messageType: 'std_msgs/Bool'
        });

        gasDetectedListener.subscribe((message) => {
            console.log('Gas Detected:', message.data);
            this.setState({ gasDetected: message.data, gasConnected: true });
        });

        const motorStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor_status',
            messageType: 'std_msgs/String'
        });

        motorStatusListener.subscribe((message) => {
            console.log('Motor Status:', message.data);
            this.setState({ motorStatus: message.data, motorConnected: true });
        });

        const alarmStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/alarm_status',
            messageType: 'std_msgs/String'
        });

        alarmStatusListener.subscribe((message) => {
            console.log('Alarm Status:', message.data);
            this.setState({ alarmStatus: message.data, alarmConnected: true });
        });

        // Add a listener for visual alerts
        const visualAlertListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/visual_alert',
            messageType: 'std_msgs/String'
        });

        visualAlertListener.subscribe((message) => {
            console.log('Visual Alert:', message.data);
            this.setState({ visualAlert: message.data });
        });
    }

    setupCameraListener() {
        const imageListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/camera/image_raw',
            messageType: 'sensor_msgs/Image'
        });

        imageListener.subscribe((message) => {
            console.log('Camera Image Received');
            const imageUrl = `data:image/jpeg;base64,${this.arrayBufferToBase64(message.data.data)}`;
            this.setState({ cameraImage: imageUrl });
        });
    }

    arrayBufferToBase64(buffer) {
        let binary = '';
        const bytes = new Uint8Array(buffer);
        const len = bytes.byteLength;

        for (let i = 0; i < len; i++) {
            binary += String.fromCharCode(bytes[i]);
        }

        return window.btoa(binary);
    }

    sendMotorCommand = (command) => {
        const motorCommandPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor_command',
            messageType: 'std_msgs/String'
        });

        const message = new ROSLIB.Message({
            data: command
        });

        motorCommandPublisher.publish(message);
    }

    render() {
        const { temperatures, pressures, gasDetected, motorStatus, alarmStatus, cameraImage, rosConnected, gasConnected, motorConnected, alarmConnected, visualAlert } = this.state;

        return (
            <div>
                <h1>Robot Dashboard</h1>

                <Alert variant={rosConnected.includes(true) ? 'success' : 'danger'}>
                    {rosConnected.includes(true) ? 'Connected to ROS' : 'Disconnected from ROS'}
                </Alert>

                <h2>Temperature Sensors</h2>
                {temperatures.map((temp, index) => (
                    <Alert key={`temp-${index}`} variant={temp > 80 ? 'danger' : (temp < 20 ? 'info' : 'success')}>
                        {`Temperature Sensor ${index + 1}: `}
                        {temp !== null ? `${temp} °C` : <Spinner animation="border" size="sm" />}
                        <br />
                        {rosConnected[index] ? 'Connected ' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Pressure Sensors</h2>
                {pressures.map((pressure, index) => (
                    <Alert key={`pressure-${index}`} variant={pressure !== null ? 'success' : 'danger'}>
                        {`Pressure Sensor ${index + 1}: `}
                        {pressure !== null ? `${pressure} units` : <Spinner animation="border" size="sm" />}
                        <br />
                        {rosConnected[index + 5] ? 'Connected' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Gas</h2>
                <Alert variant={gasDetected ? 'danger' : 'success'}>
                    Gas Detection: {gasDetected !== null ? (gasDetected ? 'Detected' : 'Not Detected') : <Spinner animation="border" size="sm" />}
                    <br />
                    {gasConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                <h2>Motor</h2>
                <Alert variant={motorStatus === 'running' ? 'success' : 'danger'}>
                    Motor Status: {motorStatus !== 'unknown' ? motorStatus : <Spinner animation="border" size="sm" />}
                    <br />
                    {motorConnected ? 'Connected' : 'Not Connected'}
                </Alert>
                <Button variant="primary" onClick={() => this.sendMotorCommand('turn_on')}>Turn On Motor</Button>
                <Button variant="danger" onClick={() => this.sendMotorCommand('turn_off')}>Turn Off Motor</Button>

                <h2>Alarm</h2>
                <Alert variant={alarmStatus === 'active' ? 'danger' : 'success'}>
                    Alarm Status: {alarmStatus !== 'inactive' ? alarmStatus : <Spinner animation="border" size="sm" />}
                    <br />
                    {alarmConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                {cameraImage ? (
                    <div>
                        <h2>Live Camera Feed</h2>
                        <img src={cameraImage} alt="Camera Feed" style={{ maxWidth: '100%' }} />
                    </div>
                ) : (
                    <Spinner animation="border" />
                )}

               
                {visualAlert && (
                    <Alert variant="warning">
                        {visualAlert}
                    </Alert>
                )}
            </div>
        );
    }
}

export default Dashboard;
*/
/*

import React, { Component } from 'react';
import ROSLIB from 'roslib';
import Alert from 'react-bootstrap/Alert';
import Button from 'react-bootstrap/Button';


class Dashboard extends Component {
    constructor(props) {
        super(props);
        this.state = {
            temperatures: Array(5).fill(null),
            pressures: Array(5).fill(null),
            gasDetected: null,
            motorStatus: 'unknown',
            alarmStatus: 'inactive',
            cameraImage: null,
            rosConnected: Array(10).fill(false),
            gasConnected: false,
            motorConnected: false,
            alarmConnected: false,
            visualAlert: ''  // State to hold visual alert messages
        };
        this.ros = new ROSLIB.Ros();
    }

    componentDidMount() {
        this.connectRos();
    }

    connectRos() {
        this.ros.on('connection', () => {
            console.log('Connected to ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => true) });
            this.setupListeners();
            this.setupCameraListener();
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to ROS:', error);
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        this.ros.on('close', () => {
            console.log('Disconnected from ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        // Adjust according to your ROSBridge server address
        this.ros.connect('ws://192.168.0.119:9090');
    }

    setupListeners() {
        for (let i = 0; i < 5; i++) {
            const temperatureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/temperature_${i}`,
                messageType: 'std_msgs/Float64'
            });

            temperatureListener.subscribe((message) => {
                console.log(`Temperature ${i}:`, message.data);
                const newTemperatures = [...this.state.temperatures];
                newTemperatures[i] = message.data;
                this.setState({ temperatures: newTemperatures });
            });

            const pressureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/pressure_${i}`,
                messageType: 'std_msgs/Float64'
            });

            pressureListener.subscribe((message) => {
                console.log(`Pressure ${i}:`, message.data);
                const newPressures = [...this.state.pressures];
                newPressures[i] = message.data;
                this.setState({ pressures: newPressures });
            });
        }

        const gasDetectedListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/gas_detected',
            messageType: 'std_msgs/Bool'
        });

        gasDetectedListener.subscribe((message) => {
            console.log('Gas Detected:', message.data);
            this.setState({ gasDetected: message.data, gasConnected: true });
        });

        const motorStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor_status',
            messageType: 'std_msgs/String'
        });

        motorStatusListener.subscribe((message) => {
            console.log('Motor Status:', message.data);
            this.setState({ motorStatus: message.data, motorConnected: true });
        });

        const alarmStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/alarm_status',
            messageType: 'std_msgs/String'
        });

        alarmStatusListener.subscribe((message) => {
            console.log('Alarm Status:', message.data);
            this.setState({ alarmStatus: message.data, alarmConnected: true });
        });

        // Add a listener for visual alerts
        const visualAlertListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/visual_alert',
            messageType: 'std_msgs/String'
        });

        visualAlertListener.subscribe((message) => {
            console.log('Visual Alert:', message.data);
            this.setState({ visualAlert: message.data });
        });
    }

    setupCameraListener() {
        const imageListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/camera/image_raw',
            messageType: 'sensor_msgs/Image'
        });

        imageListener.subscribe((message) => {
            console.log('Camera Image Received');
            const imageUrl = `data:image/jpeg;base64,${this.arrayBufferToBase64(message.data.data)}`;
            this.setState({ cameraImage: imageUrl });
        });
    }

    arrayBufferToBase64(buffer) {
        let binary = '';
        const bytes = new Uint8Array(buffer);
        const len = bytes.byteLength;

        for (let i = 0; i < len; i++) {
            binary += String.fromCharCode(bytes[i]);
        }

        return window.btoa(binary);
    }

    sendMotorCommand = (command) => {
        const motorCommandPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor_command',
            messageType: 'std_msgs/String'
        });

        const message = new ROSLIB.Message({
            data: command
        });

        motorCommandPublisher.publish(message);
    }

    render() {
        const { temperatures, pressures, gasDetected, motorStatus, alarmStatus, cameraImage, rosConnected, gasConnected, motorConnected, alarmConnected, visualAlert } = this.state;

        return (
            <div>
                <h1>Robot Dashboard</h1>

                <Alert variant={rosConnected.includes(true) ? 'success' : 'danger'}>
                    {rosConnected.includes(true) ? 'Connected to ROS' : 'Disconnected from ROS'}
                </Alert>

                <h2>Temperature Sensors</h2>
                {temperatures.map((temp, index) => (
                    <Alert key={`temp-${index}`} variant={temp !== null && temp > 80 ? 'danger' : (temp !== null && temp < 20 ? 'info' : 'success')}>
                        {`Temperature Sensor ${index + 1}: `}
                        {temp !== null ? `${temp} °C` : 'Null'}
                        <br />
                        {rosConnected[index] ? 'Connected ' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Pressure Sensors</h2>
                {pressures.map((pressure, index) => (
                    <Alert key={`pressure-${index}`} variant={pressure !== null ? 'success' : 'danger'}>
                        {`Pressure Sensor ${index + 1}: `}
                        {pressure !== null ? `${pressure} units` : 'Null'}
                        <br />
                        {rosConnected[index + 5] ? 'Connected' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Gas</h2>
                <Alert variant={gasDetected ? 'danger' : 'success'}>
                    Gas Detection: {gasDetected !== null ? (gasDetected ? 'Detected' : 'Not Detected') : 'Null'}
                    <br />
                    {gasConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                <h2>Motor</h2>
                <Alert variant={motorStatus === 'running' ? 'success' : 'danger'}>
                    Motor Status: {motorStatus !== 'unknown' ? motorStatus : 'Null'}
                    <br />
                    {motorConnected ? 'Connected' : 'Not Connected'}
                </Alert>
                <Button variant="primary" onClick={() => this.sendMotorCommand('turn_on')}>Turn On Motor</Button>
                <Button variant="danger" onClick={() => this.sendMotorCommand('turn_off')}>Turn Off Motor</Button>

                <h2>Alarm</h2>
                <Alert variant={alarmStatus === 'active' ? 'danger' : 'success'}>
                    Alarm Status: {alarmStatus !== 'inactive' ? alarmStatus : 'Null'}
                    <br />
                    {alarmConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                {cameraImage ? (
                    <div>
                        <h2>Live Camera Feed</h2>
                        <img src={cameraImage} alt="Camera Feed" style={{ maxWidth: '100%' }} />
                    </div>
                ) : (
                    'Null'
                )}

               
                {visualAlert && (
                    <Alert variant="warning">
                        {visualAlert}
                    </Alert>
                )}
            </div>
        );
    }
}

export default Dashboard;
*/
import React, { Component } from 'react';
import ROSLIB from 'roslib';
import Alert from 'react-bootstrap/Alert';
import Button from 'react-bootstrap/Button';

class Dashboard extends Component {
    constructor(props) {
        super(props);
        this.state = {
            temperatures: Array(5).fill(null),
            pressures: Array(5).fill(null),
            gasDetected: null,
            motorStatus: 'unknown',
            alarmStatus: 'inactive',
            cameraImage: null,
            rosConnected: Array(10).fill(false),
            gasConnected: false,
            motorConnected: false,
            alarmConnected: false,
            visualAlert: ''  // State to hold visual alert messages
        };
        this.ros = new ROSLIB.Ros();
    }

    componentDidMount() {
        this.connectRos();
    }

    connectRos() {
        this.ros.on('connection', () => {
            console.log('Connected to ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => true) });
            this.setupListeners();
            this.setupCameraListener();
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to ROS:', error);
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        this.ros.on('close', () => {
            console.log('Disconnected from ROS');
            this.setState({ rosConnected: this.state.rosConnected.map(() => false) });
        });

        // Adjust according to your ROSBridge server address
        this.ros.connect('ws://192.168.0.119:9090');
    }

    setupListeners() {
        for (let i = 0; i < 5; i++) {
            const temperatureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/temperature_${i}`,
                messageType: 'std_msgs/Float64'
            });

            temperatureListener.subscribe((message) => {
                console.log(`Temperature ${i}:`, message.data);
                const newTemperatures = [...this.state.temperatures];
                newTemperatures[i] = message.data;
                this.setState({ temperatures: newTemperatures });
            });

            const pressureListener = new ROSLIB.Topic({
                ros: this.ros,
                name: `/pressure_${i}`,
                messageType: 'std_msgs/Float64'
            });

            pressureListener.subscribe((message) => {
                console.log(`Pressure ${i}:`, message.data);
                const newPressures = [...this.state.pressures];
                newPressures[i] = message.data;
                this.setState({ pressures: newPressures });
            });
        }

        const gasDetectedListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/gas_detected',
            messageType: 'std_msgs/Bool'
        });

        gasDetectedListener.subscribe((message) => {
            console.log('Gas Detected:', message.data);
            this.setState({ gasDetected: message.data, gasConnected: true });
        });

        const motorStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor_status',
            messageType: 'std_msgs/String'
        });

        motorStatusListener.subscribe((message) => {
            console.log('Motor Status:', message.data);
            this.setState({ motorStatus: message.data, motorConnected: true });
        });

        const alarmStatusListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/alarm_status',
            messageType: 'std_msgs/String'
        });

        alarmStatusListener.subscribe((message) => {
            console.log('Alarm Status:', message.data);
            this.setState({ alarmStatus: message.data, alarmConnected: true });
        });

        // Add a listener for visual alerts
        const visualAlertListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/visual_alert',
            messageType: 'std_msgs/String'
        });

        visualAlertListener.subscribe((message) => {
            console.log('Visual Alert:', message.data);
            this.setState({ visualAlert: message.data });
        });
    }

    setupCameraListener() {
        const imageListener = new ROSLIB.Topic({
            ros: this.ros,
            name: '/camera/image_raw',
            messageType: 'sensor_msgs/Image'
        });

        imageListener.subscribe((message) => {
            console.log('Camera Image Received');
            const { data, height, width, encoding, step } = message;
            const imageData = new Uint8Array(data);
            const canvas = document.createElement('canvas');
            const ctx = canvas.getContext('2d');

            canvas.width = width;
            canvas.height = height;
            const imageDataCtx = ctx.createImageData(width, height);

            for (let i = 0; i < height; i++) {
                for (let j = 0; j < width; j++) {
                    const pixelIndex = i * step + j * 3;
                    const dataIndex = (i * width + j) * 4;

                    imageDataCtx.data[dataIndex] = imageData[pixelIndex];
                    imageDataCtx.data[dataIndex + 1] = imageData[pixelIndex + 1];
                    imageDataCtx.data[dataIndex + 2] = imageData[pixelIndex + 2];
                    imageDataCtx.data[dataIndex + 3] = 255;  // Set alpha to fully opaque
                }
            }

            ctx.putImageData(imageDataCtx, 0, 0);
            const imageUrl = canvas.toDataURL();
            this.setState({ cameraImage: imageUrl });
        });
    }

    sendMotorCommand = (command) => {
        const motorCommandPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor_command',
            messageType: 'std_msgs/String'
        });

        const message = new ROSLIB.Message({
            data: command
        });

        motorCommandPublisher.publish(message);
    }

    render() {
        const { temperatures, pressures, gasDetected, motorStatus, alarmStatus, cameraImage, rosConnected, gasConnected, motorConnected, alarmConnected, visualAlert } = this.state;

        return (
            <div>
                <h1>Robot Dashboard</h1>

                <Alert variant={rosConnected.includes(true) ? 'success' : 'danger'}>
                    {rosConnected.includes(true) ? 'Connected to ROS' : 'Disconnected from ROS'}
                </Alert>

                <h2>Temperature Sensors</h2>
                {temperatures.map((temp, index) => (
                    <Alert key={`temp-${index}`} variant={temp !== null && temp > 80 ? 'danger' : (temp !== null && temp < 20 ? 'info' : 'success')}>
                        {`Temperature Sensor ${index + 1}: `}
                        {temp !== null ? `${temp} °C` : 'Null'}
                        <br />
                        {rosConnected[index] ? 'Connected ' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Pressure Sensors</h2>
                {pressures.map((pressure, index) => (
                    <Alert key={`pressure-${index}`} variant={pressure !== null ? 'success' : 'danger'}>
                        {`Pressure Sensor ${index + 1}: `}
                        {pressure !== null ? `${pressure} units` : 'Null'}
                        <br />
                        {rosConnected[index + 5] ? 'Connected' : 'Not Connected'}
                    </Alert>
                ))}

                <h2>Gas</h2>
                <Alert variant={gasDetected ? 'danger' : 'success'}>
                    Gas Detection: {gasDetected !== null ? (gasDetected ? 'Detected' : 'Not Detected') : 'Null'}
                    <br />
                    {gasConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                <h2>Motor</h2>
                <Alert variant={motorStatus === 'running' ? 'success' : 'danger'}>
                    Motor Status: {motorStatus !== 'unknown' ? motorStatus : 'Null'}
                    <br />
                    {motorConnected ? 'Connected' : 'Not Connected'}
                </Alert>
                <Button variant="primary" onClick={() => this.sendMotorCommand('turn_on')}>Turn On Motor</Button>
                <Button variant="danger" onClick={() => this.sendMotorCommand('turn_off')}>Turn Off Motor</Button>

                <h2>Alarm</h2>
                <Alert variant={alarmStatus === 'active' ? 'danger' : 'success'}>
                    Alarm Status: {alarmStatus !== 'inactive' ? alarmStatus : 'Null'}
                    <br />
                    {alarmConnected ? 'Connected' : 'Not Connected'}
                </Alert>

                {cameraImage ? (
                    <div>
                        <h2>Live Camera Feed</h2>
                        <img src={cameraImage} alt="Camera Feed" style={{ maxWidth: '100%' }} />
                    </div>
                ) : (
                    'Null'
                )}

                {visualAlert && (
                    <Alert variant="warning">
                        {visualAlert}
                    </Alert>
                )}
            </div>
        );
    }
}

export default Dashboard;
