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

network DADCA_ACK_n20_u16_a8000m2_9
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
        @display("i=misc/sensor;p=568.72,7975.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1158.59,7844.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2460.95,7916.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2979.75,7849.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3583.24,7869.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4496.62,7841.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=163.58,7699.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=536.86,7596.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1747.74,7576.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2139.98,7713.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4122.22,7665.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4770.05,7515.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5617.78,7518.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5940.92,7725.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=797.81,7331.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2570.57,7346.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3129.95,7431.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3665.63,7478.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5233.60,7497.19");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=115.72,7088.67");
    }

}

