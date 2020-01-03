import React, { Component } from 'react'
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
    x_normalized = xOffset / (this.state.ViewWidth / 2.)
    y_normalized = yOffset / (this.state.ViewHeight / 2.)
    const msg_to_send = `control_command ${x_normalized} ${y_normalized}`
    console.log(msg_to_send);
    console.log(event.nativeEvent)
    if(this.state.is_socket_open)
      this.socket.send(msg_to_send);
  }
  onCircleRelease(event) {
    console.log("release")
    const msg_to_send = `control_command 0 0`
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

  // https://facebook.github.io/react-native/docs/touchablewithoutfeedback
  // ctrl click on TouchableOpacity to get src code, and observe onPressOut
  render() {
    return (
      <View style={styles.container}>
        <View style = {styles.circleContainer}>
          <TouchableOpacity
            style={this.state.is_socket_open ? styles.circleEnabled : styles.circleDisabled}
            onPressIn={(event) => this.onCirclePress(event)}
            onLayout={(event) => this.onLayout(event)}
            onPressOut={(event) => this.onCircleRelease(event)}
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
