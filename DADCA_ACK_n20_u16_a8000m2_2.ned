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

network DADCA_ACK_n20_u16_a8000m2_2
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
        @display("i=misc/sensor;p=1417.76,7756.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1801.81,7875.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2474.10,7923.75");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3791.49,7847.45");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5448.41,7907.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2121.33,7620.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3329.25,7669.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4403.06,7733.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4963.47,7642.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5985.06,7742.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1606.95,7456.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2839.83,7486.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3423.74,7306.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4412.33,7294.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5061.88,7267.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5575.39,7489.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=811.63,7093.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1330.75,7164.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1995.76,7234.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2476.03,7147.39");
    }

}

