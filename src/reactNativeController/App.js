import React, { Component } from 'react'
import Slider from "react-native-slider";
import {
  StyleSheet,
  TouchableOpacity,
  Text,
  View,
  Button,
} from 'react-native'

export default class App extends Component {

  constructor() {
    super();
    this.state = {
      is_socket_open: false,
      slider_value: 1,
    };
    this.socket = new WebSocket('ws://raspberrypi.local:8085/websocket');
    this.socket.onopen = () => {
      // connection opened
      this.socket.send('connection established'); // send a message
      this.setState({is_socket_open : true})
    };
  }

  onCirclePress(event) {
    // console.log(`x coord = ${event.nativeEvent.locationX} y coord = ${event.nativeEvent.locationY}`);
    // console.log(event.nativeEvent)
    // console.log(`Circle: x: ${this.state.circleCenterX}, y: ${this.state.circleCenterY}`);
    let xOffset = event.nativeEvent.locationX - this.state.circleCenterX;
    // use "-" to reverse Y axis
    let yOffset = -(event.nativeEvent.locationY - this.state.circleCenterY);
    msg_to_send = `control_command ${xOffset} ${yOffset} ${this.state.ViewWidth} ${this.state.ViewHeight} ${this.state.slider_value}`
    console.log(msg_to_send);
    console.log(event.nativeEvent)
    if(this.state.is_socket_open)
      this.socket.send(msg_to_send);
  }

  onLayout(event) {
    const layout = event.nativeEvent.layout;
    // console.log('ViewHeight:', layout.height);
    // console.log('ViewWidth:', layout.width);
    // console.log('topLeftX:', layout.x);
    // console.log('topLeftY:', layout.y);
    let ViewHeight = layout.height;
    let ViewWidth = layout.width;
    let circleCenterX = ViewWidth / 2.;
    let circleCenterY = ViewHeight / 2.
    this.setState(previousState => (
      { circleCenterX: circleCenterX, circleCenterY: circleCenterY, ViewWidth: ViewWidth, ViewHeight: ViewHeight}
    ))
  }
  onManualMode(event) {
    console.log("onManualMode");
    if(this.state.is_socket_open)
      this.socket.send("mode_command manual");
  }
  onTrainingMode(event) {
    console.log("onTrainingMode");
    if(this.state.is_socket_open)
      this.socket.send("mode_command training");
  }
  onAutonomousMode(event) {
    console.log("onAutonomousMode");
    if(this.state.is_socket_open)
      this.socket.send("mode_command autonomous");
  }


  render() {
    return (
      <View style={styles.container}>
        <View style = {styles.circleContainer}>
          <TouchableOpacity
            style={this.state.is_socket_open ? styles.circleEnabled : styles.circleDisabled}
            onPress={(event) => this.onCirclePress(event)}
            onLayout={(event) => this.onLayout(event)}
          >
          </TouchableOpacity>
        </View>
        <View style={styles.button_container}>
          <Button
          title={'Manual'}
          backgroundColor={'#FB6567'}
          onPress={(event) => this.onManualMode(event)}
          style={styles.button}
          />
          <Button
          title={'Training'}
          backgroundColor={'#FB6567'}
          onPress={(event) => this.onTrainingMode(event)}
          style={styles.button}
          />
          <Button
          title={'Autonomous'}
          backgroundColor={'#FB6567'}
          onPress={(event) => this.onAutonomousMode(event)}
          style={styles.button}
          />
        </View>
        <View style = {styles.slider}>
          <Slider
            value={this.state.slider_value}
            onValueChange={value => this.setState({ slider_value : value })}
            minimumValue = {1}
            maximumValue = {20}
            step = {1}
            thumbTouchSize={{width: 100 , height: 100}}
          />
          <Text>
          Time: {this.state.slider_value}
          </Text>
        </View>

      </View>
    )
  }
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    paddingHorizontal: 10
  },
  circleContainer: {
    flex: 3,
    justifyContent: 'center',
    alignItems: 'center',
    paddingHorizontal: 10
  },
  circleEnabled: {
    width: 350,
    height: 350,
    borderRadius: 350 / 2,
    backgroundColor: 'green'
  },
  circleDisabled: {
    width: 350,
    height: 350,
    borderRadius: 350 / 2,
    backgroundColor: 'red'
  },
  slider: {
    flex: 1,
    marginLeft: 10,
    marginRight: 10,
    alignItems: "stretch",
    width: 350,
    justifyContent: "center",
    // backgroundColor: '#ecf0f1'
  },
  button_container: {
    flex: 1,
    flexDirection: 'row',
    justifyContent: 'space-between'
  },
  button: {
    backgroundColor: 'green',
    width: '40%',
    height: 40
  },
})
