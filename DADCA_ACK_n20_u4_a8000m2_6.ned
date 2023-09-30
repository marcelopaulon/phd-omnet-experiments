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

network DADCA_ACK_n20_u4_a8000m2_6
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
        @display("i=misc/sensor;p=1503.75,7999.37");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2160.88,7941.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3182.24,7871.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4904.61,7991.45");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5450.39,7908.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=186.57,7744.53");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=887.28,7685.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1295.33,7651.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1874.13,7598.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2671.27,7710.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3725.40,7636.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4424.76,7690.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4874.38,7614.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5975.34,7725.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2.32,7396.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=565.06,7278.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1528.15,7367.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2258.67,7473.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3213.28,7488.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4085.03,7326.21");
    }

}

