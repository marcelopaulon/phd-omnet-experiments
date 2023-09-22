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

network DADCA_ACK_n20_a8000m2_0
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
        @display("i=misc/sensor;p=308.68,7986.83");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=466.55,7997.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=579.38,7991.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=773.05,7983.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=893.78,7966.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=956.19,7958.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=66.98,7920.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=275.13,7935.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=387.02,7949.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=517.40,7931.19");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=660.32,7947.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=802.61,7915.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=309.18,7880.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=436.29,7887.38");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=608.70,7889.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=673.35,7874.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=859.40,7889.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=24.54,7836.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=122.36,7857.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=226.30,7862.74");
    }

}

