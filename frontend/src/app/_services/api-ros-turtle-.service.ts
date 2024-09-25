
import { Observable, Subject } from 'rxjs';
import * as ROSLIB from 'roslib';
import { Injectable } from '@angular/core';

let turtle_position_subject: Subject<any>;
let turtlesim_operation_subject: Subject<any>;
let turtlesim_speed_subject: Subject<any>;


@Injectable({
  providedIn: 'root'
})

export class ApiRosTurtleService {

  private _ros: ROSLIB.Ros;
  private _connected = new Subject<boolean>();
  private _uri = "ws://127.0.0.1:9090";

  private _topic_pose?: ROSLIB.Topic;
  private _topic_speed?: ROSLIB.Topic;
  private _topic_commands?: ROSLIB.Topic;
  private _topic_go_point?: ROSLIB.Topic;

  constructor() {
    turtle_position_subject = new Subject<any>();
    turtlesim_operation_subject = new Subject<any>();
    turtlesim_speed_subject = new Subject<any>();
    this.connect();
  }

  ngOnDestroy(): void {
    this._ros.close();
  }

  connect() {
    try {
      this._connected.next(false);
      this._ros = new ROSLIB.Ros({});
      this._ros.connect(this._uri);
      this._ros.on('connection', (event: any) => {
        this._connected.next(true);
        this.subscribeToTopicsPosition();
        this.subscribeToTopicsSpeed(); 
        this.subscribeToTopicsCommands();
        this.subscribeToTopicsMoveToPoint();
        
      })

      this._ros.on('error', (error) => {
        this._connected.next(false);
      });

      this._ros.on('close', () => {
        this._connected.next(false);
      });

    } catch (error) {
      console.log("error");
    }
  }

  get connected(): Observable<boolean> {
    return this._connected.asObservable();
  }
  
  get turtle_position_listener() : Observable<boolean>{
    return turtle_position_subject.asObservable();
  }

  get turtlesim_operation_listener() : Observable<boolean>{
    return turtlesim_operation_subject.asObservable();
  }

  get turtlesim_speed_subject() : Observable<boolean>{
    return turtlesim_speed_subject.asObservable();
  }

  

  private subscribeToTopicsPosition() {
    this._topic_pose = new ROSLIB.Topic({ ros: this._ros, name: '/turtle1/pose', messageType: 'turtlesim/Pose' });
    this._topic_pose.subscribe((msg: String) => {
        turtle_position_subject.next({ msg });
    });
  }
  private subscribeToTopicsSpeed() {
    this._topic_speed = new ROSLIB.Topic({ ros: this._ros, name: '/turtle1/cmd_vel', messageType: 'geometry_msgs/Twist' });
    this._topic_speed.subscribe((msg: String) => {
      turtlesim_speed_subject.next({ msg });
    });
  }

  private subscribeToTopicsCommands() {
    this._topic_commands = new ROSLIB.Topic({ ros: this._ros, name: '/turtlesim_commands', messageType: 'std_msgs/String' });
    this._topic_commands.subscribe((msg: String) => {
        turtlesim_operation_subject.next(msg);
    });
  }
  private subscribeToTopicsMoveToPoint() {
    this._topic_go_point = new ROSLIB.Topic({ ros: this._ros, name: '/turtlesim_move_to_point', messageType: 'geometry_msgs/Pose2D' });
    this._topic_go_point.subscribe((msg: String) => {
        turtlesim_operation_subject.next(msg);
    });
  }
 


  public sendTurtleCommand(cmd): Observable<any> {
    try {
      var turtleStatusClient = new ROSLIB.Service({
        ros: this._ros,
        name: 'turtle_command_service',
        serviceType: 'turtle_command_service '
      });

      //Create request to send
      var request = new ROSLIB.ServiceRequest({
        command: cmd
      });

      //Send request and process result
      turtleStatusClient.callService(request, (result) => {
        
      });
      return turtle_position_subject.asObservable();
    } catch (error) {
    }
  }

  public sendTurtleStopCommand(cmd): Observable<any> {
    try {
      var turtleStatusClient = new ROSLIB.Service({
        ros: this._ros,
        name: 'turtle_command_stop_service',
        serviceType: 'turtle_command_service '
      });

      //Create request to send
      var request = new ROSLIB.ServiceRequest({
        command: cmd
      });

      //Send request and process result
      turtleStatusClient.callService(request, (result) => {
        
      });
      return turtle_position_subject.asObservable();
    } catch (error) {
    }
  }

  public sentTurtleMoveToPint(x,y){
    var position2D = new ROSLIB.Message({
      x : x,
      y : y,
      theta : 0
    });
    this._topic_go_point.publish(position2D);
  }
  

/*
  let str = new ROSLIB.Message({
    x: 9,
    y: 8,
    theta : 2.5
  });
  this._topic.publish(str);
*/
  
}



