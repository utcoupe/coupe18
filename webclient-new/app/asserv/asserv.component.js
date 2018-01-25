class AsservController {
  constructor (Ros) {
    this.ros = Ros.ros;



    Ros.listen('/drivers/ard_asserv/pose2d', function (e) {
      this.pushDataToChart(2, e.x);
      this.pushDataToChart(3, e.theta);
      this.pushDataToChart(4, e.y);
    }.bind(this))

    Ros.listen('/drivers/ard_asserv/speed', function (e) {
      this.pushDataToChart(0, e.pwm_speed_left);
      this.pushDataToChart(1, e.pwm_speed_right);
      this.pushDataToChart(5, e.wheel_speed_left);
      this.pushDataToChart(7, e.wheel_speed_right);
      this.pushDataToChart(6, e.linear_speed);

    }.bind(this))

    // topics to listen to
    this.topics = [
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x'
    ];

    this.options = {
      scales: {
        xAxes: [{
          ticks: {
            display: false
          },
          gridLines: {
            display: false
          }
        }]
      },
      animation: false,
      title: {
        display: true,
        text: 'Custom Chart Title',
        fontSize: 20
      }
    };

    this.datasetOverride = {
      label: 'Sinus',
      borderColor: 'rgb(75, 192, 192)',
      borderWidth: 2,
      pointRadius: 0,
      fill: false
    };

    this.charts = [];

    for (let i = 0; i < 8; i++) {

      this.charts.push({
        data: [0],
        labels: [0],
        options: JSON.parse(JSON.stringify(this.options)),
        datasetOverride: this.datasetOverride
      })
    }

    this.charts[0].options.title.text = 'PWM speed left'
    this.charts[1].options.title.text = 'PWM speed right'
    this.charts[2].options.title.text = 'X position'
    this.charts[3].options.title.text = 'Orientation'
    this.charts[4].options.title.text = 'Y position'
    this.charts[5].options.title.text = 'Wheel speed left'
    this.charts[7].options.title.text = 'Wheel speed right'
    this.charts[6].options.title.text = 'Linear speed'

    var canvas = document.getElementsByTagName('canvas')
    for (let c of canvas) { fitToContainer(c) }

    function fitToContainer (canvas) {
      // Make it visually fill the positioned parent
      canvas.style.width = '100%'
      canvas.style.height = '100%'
      // ...then set the internal size to match
      canvas.width = canvas.offsetWidth
      canvas.height = canvas.offsetHeight
    }
  }

  pushDataToChart(i, e) {
    this.charts[i].data.push(e);
    this.charts[i].labels.push(this.charts[i].labels[this.charts[i].labels.length - 1] + 0.1);
  }


  $onInit () {
    var canvas = document.querySelector('canvas')
    fitToContainer(canvas)

    function fitToContainer (canvas) {
      // Make it visually fill the positioned parent
      canvas.style.width = '100%'
      canvas.style.height = '100%'
      // ...then set the internal size to match
      canvas.width = canvas.offsetWidth
      canvas.height = canvas.offsetHeight
    }
  }
}

angular.module('roscc').component('ccAsserv', {
  templateUrl: 'app/asserv/asserv.html',
  controller: AsservController
})
