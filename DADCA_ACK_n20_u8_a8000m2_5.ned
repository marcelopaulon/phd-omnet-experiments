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

network DADCA_ACK_n20_u8_a8000m2_5
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
        @display("i=misc/sensor;p=532.16,7897.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1550.91,7797.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2218.58,7822.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2620.50,7987.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3067.48,7981.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4300.67,7884.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4805.36,7827.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5194.72,7766.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5660.50,7797.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=166.60,7520.43");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4608.06,7536.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1070.30,7483.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2694.68,7496.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3211.53,7432.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3681.15,7414.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4961.65,7444.46");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5295.15,7303.11");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=327.59,7144.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=806.92,7223.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1737.65,7199.68");
    }

}

