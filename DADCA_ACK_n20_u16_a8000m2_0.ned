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

network DADCA_ACK_n20_u16_a8000m2_0
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
        @display("i=misc/sensor;p=995.70,7924.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1335.58,7836.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2919.41,7898.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3941.25,7867.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4485.05,7791.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=411.06,7546.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1958.93,7720.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3230.56,7637.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3605.71,7721.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4836.93,7612.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1004.07,7372.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1389.32,7480.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2631.96,7437.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3559.89,7330.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3904.15,7446.83");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=83.14,7183.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=766.40,7079.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1349.41,7034.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1925.80,7066.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2850.21,7096.97");
    }

}

