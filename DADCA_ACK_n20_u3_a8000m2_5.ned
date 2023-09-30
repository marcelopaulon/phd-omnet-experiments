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

network DADCA_ACK_n20_u3_a8000m2_5
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
        @display("i=misc/sensor;p=603.80,7907.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1248.65,7868.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2486.84,7990.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3136.96,7877.77");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3637.31,7985.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4527.22,7783.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5627.68,7908.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1619.44,7710.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2198.33,7660.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3636.94,7555.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4069.48,7704.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=538.77,7466.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1190.60,7389.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1525.07,7273.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2077.51,7326.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2649.09,7496.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3266.68,7480.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4782.11,7350.66");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5162.53,7419.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=689.33,7056.06");
    }

}

