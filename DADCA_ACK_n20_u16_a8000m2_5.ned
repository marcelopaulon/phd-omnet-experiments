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

network DADCA_ACK_n20_u16_a8000m2_5
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
        @display("i=misc/sensor;p=4414.88,7853.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=5293.27,7807.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5988.15,7773.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3390.85,7543.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4103.04,7623.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4742.54,7682.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5591.68,7543.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2995.83,7434.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3800.58,7355.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4312.77,7323.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5067.32,7347.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4764.60,7132.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5916.93,7178.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3259.77,6849.00");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3822.22,6942.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4285.89,6807.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4943.45,6811.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5266.25,6990.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2104.25,6647.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2791.03,6685.78");
    }

}

