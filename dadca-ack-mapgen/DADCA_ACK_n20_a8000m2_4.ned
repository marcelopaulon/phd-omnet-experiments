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

network DADCA_ACK_n20_a8000m2_4
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
        @display("i=misc/sensor;p=219.25,7970.77");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=315.04,7996.58");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=431.38,7983.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=501.84,7976.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=568.90,7991.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=732.13,7994.34");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=868.03,7981.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=974.96,7987.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=61.19,7926.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=336.78,7929.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=477.54,7919.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=590.32,7933.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=781.29,7915.37");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2.41,7908.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=139.76,7905.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=277.89,7872.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=402.92,7884.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=517.73,7872.75");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=695.92,7887.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=162.07,7838.86");
    }

}

