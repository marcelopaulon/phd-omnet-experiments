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

network DADCA_ACK_n20_u8_a8000m2_4
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
        @display("i=misc/sensor;p=1025.38,7808.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1394.82,7816.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1982.42,7836.47");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2658.37,7845.68");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3652.74,7848.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4692.69,7969.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5319.80,7826.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=559.29,7686.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3111.99,7681.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4030.75,7682.92");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5731.68,7668.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=213.23,7369.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=784.68,7404.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1135.88,7426.11");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1612.67,7331.64");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2715.72,7384.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3127.96,7328.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3567.34,7435.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4387.62,7425.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5090.58,7426.79");
    }

}

