class TopicController {
  constructor($scope, $http, Settings, Quaternions, Ros) {
    this.$scope = $scope;
    this.$http = $http;
    this.setting = Settings.get();
    this.Quaternions = Quaternions;
    this.ros = Ros;
    this.isSubscribing = true;
    this.toggle = true;
  }

  $onInit() {
    this.roslibTopic = new ROSLIB.Topic({
      ros : this.ros.ros,
      name: this.topic.name,
      messageType: this.topic.type,
    });

    const path = 'app/topics/';
    this.fileName = `${path}default.html`;

    this.$scope.$watchGroup(['topic.type', 'topic.active'], () => {
      if(!this.topic.active) {
        this.fileName = `${path}disabled.html`;
        this.isSubscribing = false;
        this.toggle = false;
        return;
      }

      else if (!this.topic.type) {
        this.fileName = `${path}default.html`;
        this.toggleSubscription(false);
        return;
      }

      const fileName = `${path}${this.topic.type}.html`;
      this.$http.get(fileName).then((result) => {
        if (result.data) {
          this.fileName = fileName;
          this.toggleSubscription(false);
        }
      }, () => {});
    });
  }

  toggleSubscription(data) {
    if(!this.topic.active)
      return;
    if (!data) {
      this.roslibTopic.subscribe((message) => {
        this.message = message;
      });
    } else {
      this.roslibTopic.unsubscribe();
    }
    this.isSubscribing = !data;

  }

  publishMessage(input, isJSON) {
    const data = isJSON ? angular.fromJson(input) : input;
    const message = new ROSLIB.Message(data);
    this.roslibTopic.publish(message);
  }

}

angular.module('roscc').component('ccTopic', {
  bindings: { topic: '=' },
  template: '<ng-include src="$ctrl.fileName"></ng-include>',
  controller: TopicController,
});
