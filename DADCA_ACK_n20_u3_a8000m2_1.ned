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

network DADCA_ACK_n20_u3_a8000m2_1
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
        @display("i=misc/sensor;p=533.13,7939.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1100.98,7972.77");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2222.16,7812.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3347.02,7875.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3911.69,7941.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=554.79,7548.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=980.85,7582.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2732.76,7748.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3054.87,7574.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4756.21,7698.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5533.44,7727.67");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1449.64,7477.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1946.80,7475.07");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2353.56,7262.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4013.42,7483.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4374.20,7396.09");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5219.59,7435.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5715.65,7393.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=372.41,7041.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=773.19,7096.01");
    }

}

