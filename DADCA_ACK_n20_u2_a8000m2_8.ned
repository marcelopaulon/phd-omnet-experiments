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

network DADCA_ACK_n20_u2_a8000m2_8
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
        @display("i=misc/sensor;p=125.00,7864.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1418.08,7865.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2152.51,7876.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2661.76,7944.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3666.70,7925.68");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=421.09,7582.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=878.60,7511.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1783.92,7650.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3176.95,7691.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5045.29,7623.47");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5497.66,7748.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5921.15,7606.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1238.30,7271.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2090.08,7347.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2444.31,7489.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3671.75,7300.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4157.19,7330.51");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4651.85,7304.43");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5417.91,7315.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=563.86,7245.13");
    }

}

