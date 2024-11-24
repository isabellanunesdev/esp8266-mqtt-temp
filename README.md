# ESP8266 MQTT Temperature and Humidity monitor

![Languages used](https://img.shields.io/github/languages/count/isadfrn/esp8266-mqtt-temp?style=flat-square)
![Repository size](https://img.shields.io/github/repo-size/isadfrn/esp8266-mqtt-temp?style=flat-square)
![Last commit](https://img.shields.io/github/last-commit/isadfrn/esp8266-mqtt-temp?style=flat-square)

## About

ESP8266 project to read temperature and humidity data from a DHT11 sensor and send it to an MQTT broker.

## Run

```bash
sudo apt-get install mosquitto mosquitto-clients
sudo nano /etc/mosquitto/conf.d/default.conf

listener 1883
allow_anonymous true

sudo systemctl restart mosquitto

sudo systemctl status mosquitto

sudo ufw allow 1883/tcp

mosquitto_sub -h localhost -t "mestrado/iot/aluno/isabella/temperatura"
mosquitto_sub -h localhost -t "mestrado/iot/aluno/isabella/umidade"

./run
```

## Contributing

This repository is using [Gitflow Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow) and [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/), so if you want to contribute:

- create a branch from develop branch;
- make your contributions;
- open a [Pull Request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) to develop branch;
- wait for discussion and future approval;

I thank you in advance for any contribution.

## Status

Maintaining

## License

[MIT](./LICENSE)
