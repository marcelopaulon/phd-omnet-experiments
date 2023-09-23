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

network DADCA_ACK_n20_u4_a8000m2_5
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
        @display("i=misc/sensor;p=1160.74,7989.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1521.14,7918.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2633.35,7901.67");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3071.18,7996.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3447.42,7800.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5576.60,7760.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1.83,7636.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=417.70,7685.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=990.92,7683.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1388.57,7578.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1902.65,7685.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3803.39,7745.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4204.00,7660.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4598.35,7669.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=392.25,7266.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2349.89,7470.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2904.30,7486.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3294.49,7422.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3655.90,7329.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4043.13,7296.08");
    }

}

