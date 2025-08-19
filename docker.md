## View occupied space

```bash
docker system df
```

## Unnecessary space cleanup

```bash
  yes | docker system prune

  # Remove volumes
  docker volume ls -f dangling=true | awk 'NR > 1 {print $2}' | xargs docker volume rm
```

## Reboot Docker

```bash
  sudo systemctl daemon-reload && sudo systemctl restart docker
```

## View names of running containers

```bash
docker ps -q | xargs -n1 docker inspect --format '{{.Name}}'
```

## Kill running containers

```bash
running_containers=$(docker ps -aq)
docker container kill $running_containers
docker container rm   $running_containers
```
