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

network DADCA_ACK_n20_a8000m2_7
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
        @display("i=misc/sensor;p=256.91,7981.24");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=376.56,7964.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=474.81,7991.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=558.08,7983.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=850.83,7966.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=921.15,7960.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=994.09,7979.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=101.59,7939.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=195.27,7950.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=646.23,7946.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=772.35,7948.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=51.31,7889.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=317.79,7890.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=440.11,7905.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=559.49,7903.83");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=911.72,7894.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=984.36,7889.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=118.81,7842.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=180.96,7867.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=272.25,7848.40");
    }

}

