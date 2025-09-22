using System;
using Microsoft.EntityFrameworkCore.Migrations;
using Npgsql.EntityFrameworkCore.PostgreSQL.Metadata;

#nullable disable

namespace Hub.Migrations
{
    /// <inheritdoc />
    public partial class AddImuSamples : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.CreateTable(
                name: "ImuSamples",
                columns: table => new
                {
                    Id = table.Column<int>(type: "integer", nullable: false)
                        .Annotation("Npgsql:ValueGenerationStrategy", NpgsqlValueGenerationStrategy.IdentityByDefaultColumn),
                    ReceivedAt = table.Column<DateTime>(type: "timestamp with time zone", nullable: false),
                    RoverTimeNs = table.Column<decimal>(type: "numeric(20,0)", nullable: false),
                    QuaternionW = table.Column<float>(type: "real", nullable: false),
                    QuaternionX = table.Column<float>(type: "real", nullable: false),
                    QuaternionY = table.Column<float>(type: "real", nullable: false),
                    QuaternionZ = table.Column<float>(type: "real", nullable: false),
                    AccelX = table.Column<float>(type: "real", nullable: false),
                    AccelY = table.Column<float>(type: "real", nullable: false),
                    AccelZ = table.Column<float>(type: "real", nullable: false),
                    GyroX = table.Column<float>(type: "real", nullable: false),
                    GyroY = table.Column<float>(type: "real", nullable: false),
                    GyroZ = table.Column<float>(type: "real", nullable: false),
                    MagX = table.Column<float>(type: "real", nullable: false),
                    MagY = table.Column<float>(type: "real", nullable: false),
                    MagZ = table.Column<float>(type: "real", nullable: false),
                    Heading = table.Column<float>(type: "real", nullable: false),
                    Roll = table.Column<float>(type: "real", nullable: false),
                    Pitch = table.Column<float>(type: "real", nullable: false),
                    LinearAccelX = table.Column<float>(type: "real", nullable: false),
                    LinearAccelY = table.Column<float>(type: "real", nullable: false),
                    LinearAccelZ = table.Column<float>(type: "real", nullable: false),
                    TemperatureC = table.Column<float>(type: "real", nullable: false),
                    CalibSys = table.Column<long>(type: "bigint", nullable: false),
                    CalibGyro = table.Column<long>(type: "bigint", nullable: false),
                    CalibAccel = table.Column<long>(type: "bigint", nullable: false),
                    CalibMag = table.Column<long>(type: "bigint", nullable: false)
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_ImuSamples", x => x.Id);
                });
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropTable(
                name: "ImuSamples");
        }
    }
}
