FROM alpine:3.20

RUN apk add --no-cache bash curl openssh-client ca-certificates kubectl sshpass

WORKDIR /opt/vehicle

COPY scripts/ /opt/vehicle/scripts/
COPY config/ /opt/vehicle/config/
COPY deploy/ /opt/vehicle/deploy/
COPY docker/shared/ssh-runner/entrypoint.sh /usr/local/bin/ssh-runner-entrypoint
COPY docker/shared/ssh-runner/ssh /usr/local/bin/ssh
COPY docker/shared/ssh-runner/scp /usr/local/bin/scp

RUN mkdir -p /root/.ssh \
 && chmod 700 /root/.ssh \
 && chmod +x /usr/local/bin/ssh-runner-entrypoint /usr/local/bin/ssh /usr/local/bin/scp \
 && find /opt/vehicle/scripts -type f -name '*.sh' -exec chmod +x {} + \
 && find /opt/vehicle/deploy/shared/k8s -type f -name '*.sh' -exec chmod +x {} +

ENTRYPOINT ["/usr/local/bin/ssh-runner-entrypoint"]
