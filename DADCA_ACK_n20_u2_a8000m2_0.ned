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
        @display("i=misc/sensor;p=2124.33,7955.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2760.84,7955.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3943.21,7760.66");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5197.96,7956.77");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5625.05,7812.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1463.03,7736.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2595.57,7645.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3443.13,7652.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4861.35,7693.74");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=374.82,7438.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=808.14,7499.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2272.90,7394.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2958.88,7486.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4408.45,7495.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5135.36,7461.75");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5854.73,7278.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=981.08,7170.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1346.65,7022.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1697.78,7182.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3225.61,7242.24");
    }

}

