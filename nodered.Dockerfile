FROM nodered/node-red:latest
COPY ./scripts/nodered_entrypoint.sh /
ENTRYPOINT ["/nodered_entrypoint.sh"]