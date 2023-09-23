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

network DADCA_ACK_n20_u16_a8000m2_1
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
        @display("i=misc/sensor;p=765.21,7964.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1436.57,7929.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2248.79,7926.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3408.49,7943.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4084.50,7836.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4656.88,7879.38");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5658.10,7945.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=387.96,7612.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1265.64,7567.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2597.62,7533.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3730.63,7512.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4404.19,7577.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5401.41,7650.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5750.40,7506.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=622.95,7337.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1729.32,7466.56");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2111.58,7276.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3123.53,7342.04");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4806.66,7411.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=296.92,7144.03");
    }

}

