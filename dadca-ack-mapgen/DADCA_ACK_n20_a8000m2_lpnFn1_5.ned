package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n20_a8000m2_lpnFn1_5
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=113.06,181.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=91.70,271.61");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=17.46,338.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=22.86,435.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=94.51,464.76");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=63.91,550.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=67.64,395.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=129.00,411.81");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=191.14,450.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=150.97,516.28");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=185.90,295.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=229.80,341.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=253.77,283.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=271.01,216.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=338.70,264.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=197.42,172.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=44.55,201.43");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=131.01,329.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=255.50,444.05");
    }

}

