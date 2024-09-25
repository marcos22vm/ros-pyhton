import { Component, ElementRef, HostListener, OnInit, ViewChild } from '@angular/core';
import { ApiRosTurtleService } from 'src/app/_services/api-ros-turtle-.service';
@Component({
  selector: 'app-home',
  templateUrl: './home.component.html',
  styleUrls: ['./home.component.css']
})
export class HomeComponent implements OnInit {

  constructor(public apiRos: ApiRosTurtleService, private elementRef: ElementRef) { }

  @ViewChild('canvas', { static: true })
  canvas: ElementRef<HTMLCanvasElement>;


  backend_connected = false;
  keyboard_listened = false;
  show_popup = false;
  popup_message = "";

  turtle_x_relation = 11.08;
  turtle_y_relation = 11.08;
  turtle_theta_relation = 57.324840764;

  turtle_x;
  turtle_y;
  canvas_x;
  canvas_y;
  canvas_theta;

  turtle_speed_linear_x=0.0;
  turtle_speed_linear_y=0.0;
  turtle_speed_angular_z=0.0;

  private ctx: CanvasRenderingContext2D;
  canvas_with;
  canvas_heith;





  ngOnInit(): void {
    this.configureCanvas();

    this.apiRos.connected.subscribe(res => {
      this.backend_connected = res
      if (!res)
        this.clearCanvas()
    });

    this.apiRos.turtlesim_speed_subject.subscribe(res => this.printSpeed(res));

    this.apiRos.turtle_position_listener.subscribe(res => this.printNewPositionOnCanvas(res));

    this.apiRos.turtlesim_operation_listener.subscribe(res => this.receiveCommandMessage(res));
  }

  configureCanvas() {
    let elementCanvas = this.canvas.nativeElement;
    this.ctx = elementCanvas.getContext('2d');

    let display_width = elementCanvas.width;
    let display_height = elementCanvas.height;

    console.log("Medidas canvas: ", display_width, display_height)

    var downSampleSize = 2;

    elementCanvas.width = display_width * downSampleSize;
    elementCanvas.height = display_height * downSampleSize;
    this.ctx.setTransform(downSampleSize, 0, 0, downSampleSize, 0, 0);
    this.ctx.scale(0.5, 0.5);
  }

  ngAfterViewInit(): void {
    this.canvas_heith = this.canvas.nativeElement.height;
    this.canvas_with = this.canvas.nativeElement.width;
  }

  tryConnectToServer() {
    this.apiRos.connect();
  }

  printNewPositionOnCanvas(pos) {
    try {
      
      const position = pos.msg;
      let new_x_porc = parseFloat(position.x) / this.turtle_x_relation;
      let new_y_porc = parseFloat(position.y) / this.turtle_y_relation;

      let new_theta_porc = parseFloat(position.theta) * this.turtle_theta_relation;

      let new_x = this.canvas_with * new_x_porc;
      let new_y = this.canvas_heith * (1 - new_y_porc);
      console.log()

      if (new_theta_porc < 0)
        new_theta_porc = 360 - Math.abs(new_theta_porc)
      this.canvas_theta = new_theta_porc;

      //if is the first cicle, set x and y on the actual location of turtle
      if (this.canvas_x == undefined || this.canvas_y == undefined) {
        this.canvas_x = new_x;
        this.canvas_y = new_y;

        this.turtle_x=new_x;
        this.turtle_y=new_y;
        return
      }

      this.ctx.beginPath();
      this.ctx.moveTo(this.canvas_x, this.canvas_y);
      this.ctx.lineTo(new_x, new_y);
      this.ctx.stroke();

      this.canvas_x = new_x;
      this.canvas_y = new_y;
      this.turtle_x=new_x;
      this.turtle_y=new_y;

    } catch (error) {
      console.log("Error: ", error)
    }
  }

  senCommandTurtleSim(order: string) {
    this.apiRos.sendTurtleCommand(order);
  }
  senCommandStopTurtleSim(order: string) {
    this.apiRos.sendTurtleStopCommand(order);
  }

  @HostListener('document:keypress', ['$event'])
  handleKeyboardEvent(event: KeyboardEvent) {
    if (!this.keyboard_listened) return
    let key = event.key.toLowerCase();

    switch (key) {

      case "w": { this.apiRos.sendTurtleCommand("w"); break; }
      case "s": { this.apiRos.sendTurtleCommand("s"); break; }
      case "q": { this.apiRos.sendTurtleCommand("a"); break; }
      case "e": { this.apiRos.sendTurtleCommand("d"); break; }

      case "d": { this.apiRos.sendTurtleCommand("spinr"); break; }
      case "a": { this.apiRos.sendTurtleCommand("spinl"); break; }

      case "m": { this.apiRos.sendTurtleCommand("draw_star"); break; }
      case "n": { this.apiRos.sendTurtleCommand("draw_circle"); break; }

      case "z": { this.apiRos.sendTurtleStopCommand("clear"); break; }
      case "x": { this.apiRos.sendTurtleStopCommand("reset"); break; }
      case "c": { this.apiRos.sendTurtleStopCommand("pause"); break; }
    }
  }

  changeStateKeboardListen() {
    this.keyboard_listened = !this.keyboard_listened;
  }

  receiveCommandMessage(res) {
    let message_code = res.data;

    switch (message_code) {
      case "pause": {
        this.showPopupMessage("PAUSED");
        break;
      }

      case "clear": {
        this.showPopupMessage("CLEARED");
        this.clearCanvas();
        break;
      }

      case "reset": {
        this.showPopupMessage("RESETED");
        this.clearCanvas();
        break;
      }
    }
  }

  showPopupMessage(message) {
    this.popup_message = message;
    this.show_popup = true;
    setTimeout(() => this.show_popup = false, 2000);
  }

  clearCanvas() {
    this.ctx.clearRect(0, 0, this.canvas.nativeElement.width, this.canvas.nativeElement.height);
    this.turtle_x = this.canvas_with /2;
    this.turtle_y = this.canvas_heith /2;
    this.canvas_x=undefined
    this.canvas_y=undefined
    
    this.canvas_theta = 0;
  }
  

  @ViewChild('numbre_x') numbre_x: ElementRef;
  @ViewChild('numbre_y') numbre_y: ElementRef;

  moveTurtleToPoint() {

    let x = parseFloat(this.numbre_x.nativeElement.value) / this.canvas_with * this.turtle_x_relation;
    let y = parseFloat(this.numbre_y.nativeElement.value) / (this.canvas_heith * 2) * this.turtle_y_relation;

    this.apiRos.sentTurtleMoveToPint(x, y);

  }

  printSpeed(msg) {
    try {
      const speed = msg.msg;
      this.turtle_speed_linear_x = speed.linear.x;
      this.turtle_speed_linear_y = speed.linear.y;
      this.turtle_speed_angular_z = speed.angular.z
    } catch (error) {
      console.log(error)
    }

  }

  ngOnDestroy(): void {
  }



}
