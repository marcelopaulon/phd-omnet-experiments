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

network DADCA_ACK_n20_u16_a8000m2_1
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
        @display("i=misc/sensor;p=375.63,7946.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=709.24,7801.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1179.26,7760.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1924.47,7892.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2307.40,7888.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2980.15,7949.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3295.78,7780.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4439.75,7990.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5710.29,7774.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=60.87,7530.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1557.18,7752.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2660.27,7664.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4027.96,7737.24");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4712.32,7730.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5352.55,7700.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=935.16,7473.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1417.39,7422.06");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1832.52,7414.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3031.55,7497.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3485.31,7387.80");
    }

}

