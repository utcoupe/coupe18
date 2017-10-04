
class AsservController {
  constructor(Ros) {
    this.ros = Ros;

    Ros.listen('/asserv/test', function(e) {
      for(let i = 0; i < 8; i++) {
        this.charts[i].data.push(e.data);
        this.charts[i].labels.push(this.charts[i].labels[this.charts[i].labels.length-1] + 0.1);
      }
    }.bind(this));

    // topics to listen to
    this.topics = [
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
      '/asserv/x',
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
      borderColor:"rgb(75, 192, 192)",
      borderWidth: 2,
      pointRadius: 0,
      fill: false
    };


    this.charts = [];

    for(let i = 0; i < 8; i++) {
      let d = [], l = [];

      for(let y = 0; y < 10; y+=0.1) {
        d.push(Math.sin(y));
        l.push(y);
      }

      this.charts.push({
        data: d,
        labels: l,
        options: JSON.parse(JSON.stringify(this.options)),
        datasetOverride: this.datasetOverride
      })
    }

    this.charts[0].options.title.text = 'Linear Speed';
    this.charts[1].options.title.text = 'Angular Speed';
    this.charts[2].options.title.text = 'X Position';
    this.charts[3].options.title.text = 'Orientation';
    this.charts[4].options.title.text = 'Y Position';



    var canvas = document.getElementsByTagName("canvas");
    for(let c of canvas)
      fitToContainer(c);

    function fitToContainer(canvas){
      // Make it visually fill the positioned parent
      canvas.style.width ='100%';
      canvas.style.height='100%';
      // ...then set the internal size to match
      canvas.width  = canvas.offsetWidth;
      canvas.height = canvas.offsetHeight;
    }
  }

  setPID() {

    //TODO
  }

  $onInit() {
    var canvas = document.querySelector('canvas');
    fitToContainer(canvas);

    function fitToContainer(canvas){
      // Make it visually fill the positioned parent
      canvas.style.width ='100%';
      canvas.style.height='100%';
      // ...then set the internal size to match
      canvas.width  = canvas.offsetWidth;
      canvas.height = canvas.offsetHeight;
    }
  }
}

angular.module('roscc').component('ccAsserv', {
  templateUrl: 'app/asserv/asserv.html',
  controller: AsservController,
});
