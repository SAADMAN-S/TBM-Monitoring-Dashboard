/*
import React, { Component } from 'react';
import ROSLIB from 'roslib';
import Alert from "react-bootstrap/Alert";
import Config from "../scripts/Config";

class Connection extends Component {
    constructor() {
        super();
        this.state = {
            connected: false,
            ros: null
        };
    }

    componentDidMount() {
        this.initConnection();
    }

    initConnection() {
        const ros = new ROSLIB.Ros();

        ros.on("connection", () => {
            console.log("Connection established");
            this.setState({ connected: true });
        });

        ros.on("close", () => {
            console.log("Connection is closed");
            this.setState({ connected: false });

            // Attempt to reconnect after 3 seconds
            setTimeout(() => {
                try {
                    ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
                } catch (error) {
                    console.error("Connection problem:", error);
                }
            }, 3000);
        });

        ros.on("error", (error) => {
            console.error("ROS connection error:", error);
            // Handle error here, e.g., show alert to user
        });

        try {
            ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
            this.setState({ ros: ros });
        } catch (error) {
            console.error("Initial connection problem:", error);
        }
    }

    render() {
        return (
            <div>
                <Alert className="text-center m-3" variant={this.state.connected ? "success" : "danger"}>
                    {this.state.connected ? "Robot Connected" : "Robot Disconnected"}
                </Alert>
            </div>
        );
    }
}

export default Connection;
*/
import React, { Component } from 'react';
import ROSLIB from 'roslib';
import Alert from "react-bootstrap/Alert";
import Config from "../scripts/Config";

class Connection extends Component {
    constructor() {
        super();
        this.state = {
            connected: false,
            ros: null
        };
    }

    componentDidMount() {
        this.initConnection();
    }

    initConnection() {
        const ros = new ROSLIB.Ros();
        this.setState({ ros });

        ros.on("connection", () => {
            console.log("Connection established");
            this.setState({ connected: true });
        });

        ros.on("error", (error) => {
            console.error("Connection error:", error);
        });

        ros.on("close", () => {
            console.log("Connection is closed");
            this.setState({ connected: false });

            setTimeout(() => {
                try {
                    ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
                    
                } catch (error) {
                    console.error("Reconnection problem:", error);
                }
            }, 3000);
        });

        try {
            ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
        } catch (error) {
            console.error("Initial connection problem:", error);
        }
    }

    render() {
        return (
            <div>
                <Alert className="text-center m-3" variant={this.state.connected ? "success" : "danger"}>
                    {this.state.connected ? "Robot Connected" : "Robot Disconnected"}
                </Alert>
            </div>
        );
    }
}

export default Connection;
