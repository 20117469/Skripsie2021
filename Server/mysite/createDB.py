import sqlite3
from sqlite3 import Error


def create_connection(db_file):
    """ create a database connection to the SQLite database
        specified by db_file
    :param db_file: database file
    :return: Connection object or None
    """
    conn = None
    try:
        conn = sqlite3.connect(db_file)
        return conn
    except Error as e:
        print(e)

    return conn


def create_table(conn, create_table_sql):
    """ create a table from the create_table_sql statement
    :param conn: Connection object
    :param create_table_sql: a CREATE TABLE statement
    :return:
    """
    try:
        c = conn.cursor()
        c.execute(create_table_sql)
    except Error as e:
        print(e)

def main():
    database = r"wildfiresqlite.db"

    sql_create_cluster_gateway_table = """CREATE TABLE IF NOT EXISTS clusterGatewayTable (
                                            gatewayID integer PRIMARY KEY,
                                            longtitude real NOT NULL,
                                            latitude real NOT NULL,
                                            loraSamplingChannel integer NOT NULL
                                        ); """

    sql_create_my_data_table = """CREATE TABLE IF NOT EXISTS myData (
                                    id integer PRIMARY KEY,
                                    gatewayID integer NOT NULL,
                                    segmentNumber integer NOT NULL,
                                    sensorID integer NOT NULL,
                                    temperature real NOT NULL,
                                    relativeHumidity real NOT NULL,
                                    batLevel real NOT NULL,
                                    lastUpdated datetime NOT NULL,
                                    FOREIGN KEY (gatewayID) REFERENCES clusterGatewayTable (gatewayID)
                                    );"""

    sql_create_fire_alert_table = """CREATE TABLE IF NOT EXISTS fireAlert (
                                        id integer PRIMARY KEY,
                                        gatewayID integer NOT NULL,
                                        segmentNumber integer NOT NULL,
                                        sensorID integer NOT NULL,
                                        timeDetected datetime NOT NULL,
                                        FOREIGN KEY (gatewayID) REFERENCES clusterGatewayTable (gatewayID)
                                );"""

    # create a database connection
    conn = create_connection(database)

    # create tables
    if conn is not None:
        # create cluster gateway table
        create_table(conn, sql_create_cluster_gateway_table)

        # create my data table
        create_table(conn, sql_create_my_data_table)

        #create fire alert table
        create_table(conn, sql_create_fire_alert_table)
    else:
        print("Error! cannot create the database connection.")


if __name__ == '__main__':
    main()