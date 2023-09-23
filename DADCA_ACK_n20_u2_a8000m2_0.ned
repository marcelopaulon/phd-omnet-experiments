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

network DADCA_ACK_n20_u2_a8000m2_0
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
        @display("i=misc/sensor;p=1291.81,7850.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3576.27,7771.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4128.09,7828.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5470.68,7877.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5853.54,7759.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=700.28,7709.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1955.15,7747.68");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3891.01,7543.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4505.82,7515.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4979.36,7509.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1681.52,7339.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2390.63,7306.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3063.94,7405.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=494.24,7076.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1125.96,7228.83");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3159.46,7028.30");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3664.24,7203.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4020.85,7042.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5468.28,7135.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5839.59,7220.48");
    }

}

