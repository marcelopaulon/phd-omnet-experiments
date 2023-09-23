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

network DADCA_ACK_n20_u16_a8000m2_6
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
        @display("i=misc/sensor;p=243.37,7837.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1217.14,7884.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1631.57,7951.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2617.07,7941.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3767.49,7825.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4802.00,7808.51");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=615.84,7674.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2331.63,7523.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4177.71,7714.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5321.28,7731.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5710.72,7542.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=146.52,7329.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1262.92,7437.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1694.72,7337.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2929.66,7358.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3379.50,7471.57");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3837.56,7341.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4416.90,7273.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4933.20,7324.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=397.76,7063.14");
    }

}

