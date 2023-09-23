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

network DADCA_ACK_n20_u4_a8000m2_2
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
        @display("i=misc/sensor;p=585.28,7912.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1280.27,7828.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2319.04,7997.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3253.18,7958.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3940.37,7782.49");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4648.04,7860.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5035.50,7841.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5647.57,7937.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=85.04,7530.40");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=926.10,7681.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1812.28,7715.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2255.03,7561.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2941.22,7713.82");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3504.91,7557.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5915.49,7649.24");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=704.66,7309.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1579.11,7442.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2634.49,7379.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3863.82,7385.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4471.24,7445.87");
    }

}

