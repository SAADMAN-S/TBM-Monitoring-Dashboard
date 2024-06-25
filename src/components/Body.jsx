import React, { Component } from "react";
import { Container } from "react-bootstrap";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import Home from "./Home";
import About from "./About";
import Dashboard from "./Dashboard";

class Body extends Component {
    state = {};

    render() {
        return ( 
            <Container>
                <Router>
                    <Routes>
                        <Route path="/" element={<Home />} />
                        <Route path="/about" element={<About />} />
                        <Route path="/dashboard" element={<Dashboard />} />
                    </Routes>
                </Router>
            </Container>
        );
    }
}

export default Body;