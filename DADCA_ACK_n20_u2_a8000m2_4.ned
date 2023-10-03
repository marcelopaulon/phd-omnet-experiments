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

network DADCA_ACK_n20_u2_a8000m2_4
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
        @display("i=misc/sensor;p=963.44,7783.36");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2614.61,7983.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2971.47,7896.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3737.65,7760.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5037.78,7764.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5452.93,7804.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5756.85,7980.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=11.47,7583.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1565.09,7727.88");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2200.45,7689.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2607.83,7542.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3148.13,7536.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4262.97,7610.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4636.76,7529.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1079.71,7386.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1667.18,7307.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3522.82,7447.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5818.46,7424.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=601.28,7252.99");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2105.52,7208.50");
    }

}

