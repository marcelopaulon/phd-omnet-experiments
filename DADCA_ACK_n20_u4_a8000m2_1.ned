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

network DADCA_ACK_n20_u4_a8000m2_1
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
        @display("i=misc/sensor;p=1178.34,7855.35");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2511.23,7843.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3660.50,7965.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4608.06,7931.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5694.54,7789.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=632.16,7576.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2030.91,7613.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2870.77,7750.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4243.88,7659.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5271.99,7705.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1303.82,7386.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2344.76,7363.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2847.24,7358.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3263.82,7404.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3665.39,7326.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4563.19,7409.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4939.54,7363.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=312.97,7192.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=999.25,7179.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2069.07,7032.76");
    }

}

