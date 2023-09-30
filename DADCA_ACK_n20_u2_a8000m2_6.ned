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

network DADCA_ACK_n20_u2_a8000m2_6
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
        @display("i=misc/sensor;p=358.48,7839.37");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=898.13,7790.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2889.47,7943.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3249.67,7885.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3751.60,7956.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4425.73,7913.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5042.93,7847.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1306.30,7619.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1780.14,7733.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2243.98,7672.41");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4616.54,7566.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5572.08,7726.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5964.46,7542.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=488.08,7466.63");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2066.98,7261.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2745.64,7314.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3117.03,7338.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3530.97,7476.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4163.58,7438.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5013.63,7323.07");
    }

}

