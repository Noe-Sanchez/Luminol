import express from "express";
import ViteExpress from "vite-express";

const app = express();
app.use(express.json());

let telemetryData; // json telemetry data

ViteExpress.listen(app, 19000, () => console.log("Server is listening..."));

// Telemetry routes

app.get("/telemetry", (req, res) => {
  res.json(telemetryData);
});

app.post("/telemetry", (req, res) => {
  telemetryData = req.body;
});
