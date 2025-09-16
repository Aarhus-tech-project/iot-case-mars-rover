namespace Hub.Models
{
    public class CommandMessage
    {
        public int Id { get; set; }
        public string? CommandText { get; set; }      // Command sent from website
        public string? ReplyText { get; set; }        // Response from Rover
        public int StatusId { get; set; }              // Foreign key to CommandStatus
        public CommandStatus? Status { get; set; }      // Pending, Sent, Received, Failed
        public DateTime CommandSentAt { get; set; } = DateTime.UtcNow;
        public DateTime? ReplyReceivedAt { get; set; } // Nullable, filled when reply comes
    }
}
