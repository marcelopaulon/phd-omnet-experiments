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

network DADCA_ACK_n20_u4_a8000m2_5
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
        @display("i=misc/sensor;p=1337.25,7880.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1721.34,7990.77");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2398.52,7940.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3303.93,7931.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5528.97,7945.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5867.87,7836.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=33.09,7642.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=650.09,7658.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1852.46,7517.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2913.48,7725.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3948.93,7680.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4348.21,7541.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4926.43,7586.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5361.67,7583.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=283.81,7271.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3202.99,7353.47");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3616.83,7487.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5738.10,7441.79");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=878.43,7129.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1309.71,7249.59");
    }

}

