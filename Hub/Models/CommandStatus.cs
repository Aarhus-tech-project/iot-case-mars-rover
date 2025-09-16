namespace Hub.Models
{
    public class CommandStatus
    {
        public int Id { get; set; }
        public required string Description { get; set; }     // e.g., "Pending", "Sent", "Received", "Failed"
        
        public ICollection<CommandMessage> Commands { get; set; } = new List<CommandMessage>(); // Navigation property
    }
}
