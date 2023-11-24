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

network DADCA_ACK_n20_u8_a8000m2_8
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
        @display("i=misc/sensor;p=2084.52,7793.19");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2538.30,7942.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3404.40,7838.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3776.98,7949.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4427.01,7900.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5166.04,7916.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=654.10,7625.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1148.40,7696.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1510.01,7572.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2951.57,7608.19");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3869.82,7558.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1758.79,7304.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2255.35,7300.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2612.55,7423.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4435.54,7422.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5017.63,7345.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5365.24,7392.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=265.94,7171.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=759.16,7222.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1507.55,7033.38");
    }

}

