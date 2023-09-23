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

network DADCA_ACK_n20_u2_a8000m2_4
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
        @display("i=misc/sensor;p=804.63,7776.10");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1315.10,7956.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1790.54,7836.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2157.59,7839.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3408.06,7891.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4258.21,7845.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5712.38,7815.41");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=490.13,7613.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2648.41,7557.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3815.64,7688.10");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5173.79,7628.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1028.11,7476.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1689.77,7448.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2067.04,7452.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3267.61,7391.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4642.64,7387.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5584.21,7257.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=315.46,7215.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=913.48,7028.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1486.80,7120.17");
    }

}

