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
        @display("i=misc/sensor;p=218.41,7994.74");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1369.27,7888.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1736.80,7973.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2282.12,7896.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2683.57,7968.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4980.15,7977.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5823.74,7979.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=505.68,7593.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2682.60,7506.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2982.07,7737.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4231.24,7633.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4808.35,7558.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=34.66,7430.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1817.10,7413.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2323.70,7373.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3860.62,7487.94");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5511.56,7436.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5840.51,7289.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=393.84,7215.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1330.07,7058.86");
    }

}

