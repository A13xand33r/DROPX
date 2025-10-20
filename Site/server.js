import express from "express";
import cors from "cors";
import dotenv from "dotenv";
import { Resend } from "resend";

dotenv.config();

const app = express();
app.use(cors());
app.use(express.json({ limit: "15mb" }));

const resend = new Resend(process.env.RESEND_API_KEY);

app.post("/send-email", async (req, res) => {
  const { to, subject, text, qrBase64 } = req.body;
  if (!to || !subject || !text || !qrBase64) {
    return res.status(400).json({ error: "Missing fields" });
  }

  try {
    // Ensure it’s raw base64, no prefix
    const cleanBase64 = qrBase64.replace(/^data:image\/jpeg;base64,|^data:image\/jpg;base64,/, "");

    const html = `
      <h2>DropX Delivery</h2>
      <p>${text}</p>
      <p>Your QR code is attached as <b>delivery_qr.jpg</b>.</p>
    `;

    const response = await resend.emails.send({
      from: `DropX <${process.env.FROM_EMAIL}>`,
      to,
      subject,
      html,
      text,
      attachments: [
        {
          filename: "delivery_qr.jpg",
          content: cleanBase64,
          type: "image/jpeg", // <-- very important
        },
      ],
    });

    res.json({ message: "Email sent", response });
  } catch (err) {
    console.error("Email send error:", err);
    res.status(500).json({ error: "Failed to send email" });
  }
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => console.log(`🚀 Server running on http://localhost:${PORT}`));
