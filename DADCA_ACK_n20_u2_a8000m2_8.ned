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

network DADCA_ACK_n20_u2_a8000m2_8
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
        @display("i=misc/sensor;p=1313.73,7789.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3038.64,7903.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3674.55,7800.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4165.64,7996.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4740.04,7910.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5473.73,7972.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=756.47,7547.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2109.06,7664.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4216.94,7561.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=232.24,7421.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1161.16,7310.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1516.79,7403.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2438.50,7389.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3437.74,7391.84");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4586.34,7464.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4967.96,7496.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5405.06,7384.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1902.37,7062.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4277.37,7053.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5983.44,7110.84");
    }

}

