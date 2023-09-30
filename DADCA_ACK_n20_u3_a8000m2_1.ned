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
        @display("i=misc/sensor;p=386.43,7982.74");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=820.38,7772.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1299.21,7977.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1764.85,7980.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2195.54,7883.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3245.50,7836.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=316.45,7589.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1344.80,7545.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2487.15,7596.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3913.96,7692.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4375.37,7629.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5748.07,7619.72");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=678.03,7410.34");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1922.02,7500.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2818.90,7426.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3322.46,7453.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4981.14,7480.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=455.08,7117.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1126.51,7065.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2383.44,7121.42");
    }

}

