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

network DADCA_ACK_n20_a8000m2_2
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
        @display("i=misc/sensor;p=173.98,7783.97");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=686.91,7966.58");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1107.66,7993.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1657.31,7741.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2244.95,7980.73");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2809.60,7929.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3224.18,7893.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4191.04,7913.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5350.98,7756.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5880.98,7905.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=613.93,7545.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1173.62,7490.93");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2793.49,7525.65");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4774.17,7681.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=870.39,7276.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2390.78,7376.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3616.90,7480.51");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3999.35,7378.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5479.45,7386.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5861.73,7415.91");
    }

}

