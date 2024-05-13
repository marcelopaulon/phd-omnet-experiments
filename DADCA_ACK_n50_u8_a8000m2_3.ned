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

network DADCA_ACK_n50_u8_a8000m2_3
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
        @display("i=misc/sensor;p=1400.30,7917.96");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2680.44,7864.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4360.79,7765.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=433.24,7655.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1696.20,7523.01");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3298.65,7655.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3712.04,7733.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4867.86,7547.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5712.49,7698.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=53.10,7337.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1272.42,7459.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2195.10,7437.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4265.46,7367.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5333.05,7355.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=475.97,7254.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=840.63,7221.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1708.57,7038.09");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2093.45,7075.80");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2494.28,7179.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3022.27,7090.99");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3393.73,7233.21");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3721.97,7040.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4724.39,7201.79");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=10.54,6941.56");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=426.76,6857.25");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1186.18,6814.21");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4321.24,6769.83");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5703.63,6935.36");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=242.07,6521.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=765.14,6653.58");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2420.72,6731.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3876.63,6552.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4974.87,6734.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5352.25,6605.51");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5925.34,6612.08");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1109.41,6316.12");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1923.58,6378.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2906.47,6423.67");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3386.16,6479.46");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4606.87,6309.94");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5142.33,6289.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=104.65,6088.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=612.47,6053.90");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2411.60,6221.64");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3706.11,6185.27");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4170.50,6073.22");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5690.95,6092.13");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1647.45,5849.85");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2192.08,5834.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2877.49,6015.69");
    }

}

