using Microsoft.EntityFrameworkCore;
using Hub.Models;

namespace Hub.Data
{
    public class HubDbContext : DbContext
    {
        public DbSet<CommandMessage> CommandMessages { get; set; }
        public DbSet<CommandStatus> CommandStatuses { get; set; }
        public HubDbContext(DbContextOptions<HubDbContext> options)
            : base(options)
        {
        }
        protected override void OnModelCreating(ModelBuilder modelBuilder)
        {
            base.OnModelCreating(modelBuilder);

            modelBuilder.Entity<CommandStatus>().HasData(
                new CommandStatus { Id = 1, Description = "Pending" },
                new CommandStatus { Id = 2, Description = "Sent" },
                new CommandStatus { Id = 3, Description = "Received" },
                new CommandStatus { Id = 4, Description = "Failed" }
            );
        }

        protected override void OnConfiguring(DbContextOptionsBuilder optionsBuilder)
        {
            if (!optionsBuilder.IsConfigured)
            {
                // read from config
                var config = new ConfigurationBuilder()
                    .SetBasePath(Directory.GetCurrentDirectory())
                    .AddJsonFile("appsettings.json", optional: true, reloadOnChange: true)
                    .AddJsonFile($"appsettings.Development.json", optional: true, reloadOnChange: true)   // optional for local dev
                    .AddEnvironmentVariables()
                    .Build();

                var connectionString = config.GetConnectionString("DefaultConnection");
                optionsBuilder.UseNpgsql(connectionString);
            }
        }
    }
}
