using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

#pragma warning disable CA1814 // Prefer jagged arrays over multidimensional

namespace Hub.Migrations
{
    /// <inheritdoc />
    public partial class FixCommandStatus : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.InsertData(
                table: "CommandStatuses",
                columns: new[] { "Id", "Description" },
                values: new object[,]
                {
                    { 1, "Pending" },
                    { 2, "Sent" },
                    { 3, "Received" },
                    { 4, "Failed" }
                });
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DeleteData(
                table: "CommandStatuses",
                keyColumn: "Id",
                keyValue: 1);

            migrationBuilder.DeleteData(
                table: "CommandStatuses",
                keyColumn: "Id",
                keyValue: 2);

            migrationBuilder.DeleteData(
                table: "CommandStatuses",
                keyColumn: "Id",
                keyValue: 3);

            migrationBuilder.DeleteData(
                table: "CommandStatuses",
                keyColumn: "Id",
                keyValue: 4);
        }
    }
}
