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

network DADCA_ACK_n20_u3_a8000m2_0
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
        @display("i=misc/sensor;p=805.99,7961.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1177.46,7895.35");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1558.91,7921.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1907.46,7760.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3741.59,7886.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4085.57,7988.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4934.45,7851.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1070.62,7531.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1461.99,7561.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2494.80,7550.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4513.08,7510.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5606.88,7751.80");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5970.70,7752.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3300.03,7452.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4893.56,7325.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5953.51,7367.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=547.19,7031.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1040.03,7181.24");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1593.18,7236.08");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3031.59,7154.82");
    }

}

