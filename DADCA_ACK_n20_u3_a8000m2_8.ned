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

network DADCA_ACK_n20_u3_a8000m2_8
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
        @display("i=misc/sensor;p=782.49,7911.61");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2058.16,7935.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2614.53,7902.09");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3155.71,7801.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4250.11,7811.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4833.25,7791.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5246.89,7993.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5971.16,7888.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=271.32,7545.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1095.52,7697.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1499.61,7550.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3640.52,7688.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5671.40,7576.65");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=735.94,7448.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1190.08,7288.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3018.27,7396.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3577.55,7315.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3913.50,7440.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4452.47,7421.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5118.88,7328.71");
    }

}

