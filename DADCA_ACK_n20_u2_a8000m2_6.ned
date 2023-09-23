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
        @display("i=misc/sensor;p=446.85,7926.64");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2079.64,7764.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2576.32,7904.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3039.09,7893.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4144.28,7841.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4692.64,7877.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5103.35,7921.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=753.76,7528.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1290.37,7717.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4485.74,7515.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5455.07,7505.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5977.01,7738.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=7.60,7331.83");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=387.39,7491.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1683.22,7283.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2674.22,7474.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3133.80,7488.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3798.90,7432.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5065.02,7348.89");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=462.21,7131.96");
    }

}

